#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>

static float linear_vel = 0.8;
static float angular_vel = 5;
static float max_linear_vel = 20;
static float min_linear_vel = -20;
static float max_angular_vel = 180;
static float min_angular_vel = -180;

int GetCh()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_cmd");

  printf("Keyboard controlled balance car: \n");
  printf("w - forward acceleration \n");
  printf("s - Backward acceleration \n");
  printf("a - Rotate Left \n");
  printf("d - Rotate Right \n");
  printf("Space - Brake \n");
  printf("x - Exit \n");
  printf("------------- \n");

  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/my_cmd_vel", 10);

  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;

  while(n.ok())
  {
    int cKey = GetCh();
    if(cKey=='w')
    {
      base_cmd.linear.x += linear_vel;
      if(base_cmd.linear.x > max_linear_vel) base_cmd.linear.x = max_linear_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='s')
    {
      base_cmd.linear.x += -linear_vel;

      if(base_cmd.linear.x < min_linear_vel) base_cmd.linear.x = min_linear_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='a')
    {
      base_cmd.angular.z += angular_vel;
      if(base_cmd.angular.z > max_angular_vel) base_cmd.angular.z = max_angular_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='d')
    {
      base_cmd.angular.z += -angular_vel;
      if(base_cmd.angular.z < min_angular_vel) base_cmd.angular.z = min_angular_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey==' ')
    {
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='x')
    {
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
      printf("Exit! \n");
      return 0;
    } 
    else
    {
       printf(" - Undefined Instruction \n");
    }
    
  }
  return 0;
}