extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{  
#include "roboticscape.h"
}
/*
extern "C"
{
#include "rc_motors.h"
}
*/

//#include "balance_config.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "unistd.h"

// Define Globle Variables
// Define velocity
float linear_premap = 0;
float angular_premap = 0;
float x_max = 3;
float z_max = 2;

// Define duty cycle for dual motors
unsigned int LeftChannel = 1;
unsigned int RightChannel  = 4;
float LeftDuty = 0;
float RightDuty = 0;

float median_duty = 0;
float differential_duty = 0;
float median_duty_max = 0.6;
float diff_duty_max = 0.4;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void drive_Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel_twist)
{

  // assign the commands if the array is of the correct length
  linear_premap   = cmd_vel_twist->linear.x;
  angular_premap  = cmd_vel_twist->angular.z;

//  time_last_good_ros_command_sec = ros::Time::now().toSec();
  ROS_INFO("lin, ang= %1.2f %1.2f %1.2f" , linear_premap, angular_premap);
//  return;

// Map from linear_premap & angular_premap to LeftDuty & RightDuty
// 1. Limit linear_premap to [0, x_max];
   if( linear_premap > x_max ){
   	linear_premap = x_max;
	ROS_INFO("Linear velocity upper bound excessed!");
   }
   if( linear_premap < 0 ){
        linear_premap = 0;
	ROS_INFO("Linear velocity lower bound excessed!");
   }

// 2. Map linear_premap to median_duty in the range of [0, median_duty_max];
   median_duty = median_duty_max * linear_premap / x_max;
 
// 3. limit angular_prremap to [-z_max, z_max];
   if( angular_premap > z_max ){
        angular_premap = z_max;
        ROS_INFO("Angular velocity upper bound excessed!");
   }
   if( angular_premap < (-1.0)*z_max ){
        angular_premap = (-1.0)*z_max;
        ROS_INFO("Angular velocity lower bound excessed!");
   }

// 4. Map angular_premap to differential_duty in range of [-diff_duty_max, diff_duty_max];
   differential_duty = 2.0 * diff_duty_max * (angular_premap + z_max) / (2.0 * z_max) - diff_duty_max;
// 5. Map LeftDuty = median_duty + differential_duty & RightDuty = median_duty - differential_duty
   LeftDuty = median_duty + differential_duty;
   RightDuty = median_duty - differential_duty;
//   rc_set_motor(LeftChannel, LeftDuty);
//   rc_set_motor(RightChannel, RightDuty);

   ROS_INFO("Left Duty = %1.2f , Right Duty = %1.2f" , LeftDuty, RightDuty);
/*
//  Test Motors
   rc_set_state(RUNNING);
   rc_set_motor(1, 0.2);
   rc_set_motor(4, 0.2);  
   return;
*/
}

void ros_compatible_shutdown_signal_handler(int signo)
{
  if (signo == SIGINT)
    {
      rc_set_state(EXITING);
      ROS_INFO("\nReceived SIGINT Ctrl-C.");
      ros::shutdown();
    }
  else if (signo == SIGTERM)
    {
      rc_set_state(EXITING);
      ROS_INFO("Received SIGTERM.");
      ros::shutdown();
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "psr_drive");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/turtlebot_teleop/cmd_vel", 1, drive_Callback);

  if(rc_initialize()<0)
    {
      ROS_INFO("ERROR: failed to initialize cape.");
      return -1;
}

  signal(SIGINT,  ros_compatible_shutdown_signal_handler);	
  signal(SIGTERM, ros_compatible_shutdown_signal_handler);	

  // rc_set_motor function test
//  rc_set_state(RUNNING);

//  initialize_motors();
  rc_enable_motors();
  ros::Rate r(100);  //100 hz
  while(ros::ok()){
	//rc_enable_motors();
	rc_set_motor(LeftChannel, LeftDuty);
	rc_set_motor(RightChannel, RightDuty);
	ros::spinOnce();
	r.sleep();
//	rc_usleep(10000);

}

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
//  ros::spin();

  return 0;
}

