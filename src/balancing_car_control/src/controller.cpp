#include "ros/ros.h"
#include "balancing_car_control/control_param.h"


double outerLoopIntegralOutput;                 // External loop integral output
double outerLoopIntegralMaxThre = 1;            // Integral separation threshold
double outerLoopIntegralMinThre = -1;
double outerLoopMaxIntegral = 1;                // Integral limiting amplitude
double outerLoopMinIntegral = -1;               

double yawIntegralOutput;                       // Steering loop integral output
double yawIntegralMaxThre = 10;                 // Integral separation threshold
double yawIntegralMinThre = -10;
double yawMaxIntegral = 1;                      // Integral limiting amplitude
double yawMinIntegral = -1; 


bool doPID(balancing_car_control::control_param::Request& req, balancing_car_control::control_param::Response& resp)
{
    if (req.dst_vel == 0) {outerLoopIntegralOutput = 0;}
    if (req.dst_yaw == 0) {yawIntegralOutput = 0;}

    //---------------------------------- Speed loop (outer loop) ---------------------------
    double Kp1 = 0.3;          
    double Ki1 = 0.05;         
    double velocityError = req.dst_vel - (req.leftWheelVelocity+req.rightWheelVelocity)/2; 
    // Integral separation
    if (velocityError<outerLoopIntegralMaxThre || velocityError>outerLoopIntegralMinThre) {
        outerLoopIntegralOutput += Ki1 * velocityError;
    } else {
        outerLoopIntegralOutput = 0;
    }
    // Integral limiting
    outerLoopIntegralOutput = (outerLoopIntegralOutput>outerLoopMaxIntegral)?outerLoopMaxIntegral:(outerLoopIntegralOutput < outerLoopMinIntegral) ? outerLoopMinIntegral : outerLoopIntegralOutput;
    
    double outerLoopOutput = Kp1 * velocityError + Ki1 * outerLoopIntegralOutput;
    //--------------------------------------------------------------------------------------

    //-------------------------------- Vertical loop (inner loop) ----------------------------
    double Kp2 = 0.05;          
    double Kd2 = 0.5;  
    double pitch_d = req.pitch - outerLoopOutput;                
    double pwm = Kp2 * pitch_d + Kd2 * req.pitchAngVelocity; 
    //----------------------------------------------------------------------------------------

    //------------------------------------ Steering loop ----------------------------------
    double Kp3 = 0.001;            
    double Kd3 = 0.04;             
    double Ki3 = 0.0001;             
    double yaw_d = req.yaw - req.dst_yaw;
    // Integral separation
    if (yaw_d<yawIntegralMaxThre || yaw_d>yawIntegralMinThre) {
        yawIntegralOutput += Ki3 * yaw_d;
    } else {
        yawIntegralOutput = 0;
    }
    // Integral limiting
    yawIntegralOutput = (yawIntegralOutput>yawMaxIntegral)?yawMaxIntegral:(yawIntegralOutput < yawMinIntegral) ? yawMinIntegral : yawIntegralOutput;
    
    double pwm_yaw = Kp3 * yaw_d + Ki3 * yawIntegralOutput + Kd3 * req.yawAngVelocity;
    //--------------------------------------------------------------------------------------


    // Calculate the torque values for two wheels
    resp.leftTorque = pwm + pwm_yaw;
    resp.rightTorque = pwm - pwm_yaw;

    return true;
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("control_param", doPID);
    ROS_INFO("my controller launched.....");

    ros::spin();
    return 0;
}