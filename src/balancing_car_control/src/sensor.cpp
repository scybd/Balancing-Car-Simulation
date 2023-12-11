#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "sensor_msgs/JointState.h"
#include "balancing_car_control/control_param.h"


ros::ServiceClient client;                      // Client requesting control solution
ros::Publisher wheelsCommandPub;                // Publish wheel torque

double lastPitch, lastYaw;                      // The previous Euler angle is used to obtain the angular velocity
double leftWheelVelocity, rightWheelVelocity;
double dst_vel;
double dst_yaw;


// Subscribe the speed of two rounds
void WheelVelocityCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->velocity.size() > 0)
    {
        leftWheelVelocity = msg->velocity[0]; 
        rightWheelVelocity = msg->velocity[1];
    }
}

// Subscribe keyboard control information
void KeyboardCmd(geometry_msgs::Twist msg)
{
    dst_vel = msg.linear.x;
    dst_yaw = msg.angular.z;
}


// Subscribe IMU data
void IMUCallback(sensor_msgs::Imu msg)
{
    if (msg.orientation_covariance[0] < 0) return;
    
    // Convert quaternions in message packets to quaternions in tf
    tf::Quaternion quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    
    // Calculate the posture at this time
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    // The angular velocities of pitch and yaw
    double pitchAngVelocity = pitch - lastPitch;
    double yawAngVelocity = yaw - lastYaw;
    
    // Load request message
    balancing_car_control::control_param cp;
    cp.request.dst_vel = dst_vel;
    cp.request.dst_yaw = dst_yaw;
    cp.request.leftWheelVelocity = leftWheelVelocity;
    cp.request.rightWheelVelocity = rightWheelVelocity;
    cp.request.pitch = pitch;
    cp.request.yaw = yaw;
    cp.request.yawAngVelocity = yawAngVelocity;
    cp.request.pitchAngVelocity = pitchAngVelocity;
    // Send request message
    bool flag = client.call(cp);

    // Receive the calculated torque value and publish it
    if (flag) {
        std_msgs::Float64MultiArray commandMsg;
        commandMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        commandMsg.layout.dim[0].label = "";
        commandMsg.layout.dim[0].size = 2;
        commandMsg.layout.dim[0].stride = 1;
        commandMsg.layout.data_offset = 0;
        commandMsg.data.push_back(cp.response.leftTorque);      // Left wheel, driving the car forward when the value is positive
        commandMsg.data.push_back(cp.response.rightTorque);     // Right wheel, driving the car forward when the value is positive

        wheelsCommandPub.publish(commandMsg);
    }

    lastPitch = pitch;
    lastYaw = yaw;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sensor");
    ros::NodeHandle nh;

    client = nh.serviceClient<balancing_car_control::control_param>("control_param");
    ros::service::waitForService("control_param");

    // Control the wheel output torque
    wheelsCommandPub = nh.advertise<std_msgs::Float64MultiArray>("/wheels_controller/command", 1);

    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, IMUCallback);

    // Subscription of speed information for two wheels for feedback control
    ros::Subscriber wheelVelocity_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, WheelVelocityCallback);

    ros::Subscriber keyboard_cmd_sub = nh.subscribe<geometry_msgs::Twist>("/my_cmd_vel", 10, KeyboardCmd);


    ros::spin();
    return 0;
}
