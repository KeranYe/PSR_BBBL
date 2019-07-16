include "rc_usefulincludes.h"
}
extern "C"
{  
#include "roboticscape.h"
}

//#include "balance_config.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "unistd.h"

//-----------------------------
//------ Global Variables------
//-----------------------------

//-----------------------------
// Define States: 
// pos = omega, vel = omega_dot
//-----------------------------

// State limitation
float vel_left_max = 3.5;
float vel_right_max = 3.5;

// Real states 
float pos_left = 0; 
float pos_right = 0; 

float vel_left = 0; 
float vel_right = 0; 

// Desired states 
float pos_left_des = 0; 
float pos_right_des = 0; 

float vel_left_des = 0; 
float vel_right_des = 0; 

//-----------------------------
// Define Motor Parameters
//-----------------------------

// Motor channel
unsigned int Channel_Left = 1;
unsigned int Channel_Right = 4;

// Moter mechanical property
float gear_ratio = 298.0;
float encoder_res = 12.0;

// Motor duty cycle
float duty_left = 0;
float duty_right = 0; 

//-----------------------------
// Define Controller Parameters
//-----------------------------

//PID Coeffients
float kp = 10;
float ki = 0;
float kd = 15; 


//-----------------------------
//------ Functions-------------
//-----------------------------

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}


void drive_Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel_twist)
{

	// Here we borrow geometry_msgs/Twist to store desired states: positions and velocities.
  	// Will create specific message type for PSR later.


  	//Retrieve data to global state variables
  	pos_left_des = cmd_vel_twist -> linear.x;
  	pos_right_des = cmd_vel_twist -> linear.y;
	vel_left_des = cmd_vel_twist -> angular.x;
  	vel_right_des = cmd_vel_twist -> angular.y

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


float PID_duty(float error, float integral, float derivative)
{
	// This PID aims to achieve position control.
	float duty = kp * error + ki * integral + kd * derivative

	// Limit check
	if( duty > 1 )
		duty = 1;
	else if( duty < -1 )
		duty = -1;

	return output;
}


int main(int argc, char **argv)
{
	// 1. Initialzation	
	float WheelD1=0;
	float WheelD2=0;
	float WheelD3=0;

	float error_left = 0;
	float error_right=0;

	float integral_left=0;
	float integral_right=0;
	
	float derivative_left=0;
	float derivative_right=0;

	float errorp1=0;
	float errorp2=0;
	float errorp3=0;

	float duty_left_p=0;
	float duty_right_p=0;
	float M3_Duty_p=0;
	
	ros::init(argc, argv, "psr_drive");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/turtlebot_teleop/cmd_vel", 10, drive_Callback);

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
		for (int i = 0; i < 25; ++i){

		WheelD1 = (rc_get_encoder_pos(Channel_Left) * TWO_PI)/(1 * gear_ratio * encoder_res)*0.06;
		WheelD2 = (rc_get_encoder_pos(Channel_Right) * TWO_PI)/(1 * gear_ratio * encoder_res)*0.06;
		WheelD3 = (rc_get_encoder_pos(Motor3Channel) * TWO_PI)/(1 * gear_ratio * encoder_res)*0.06;
	
		error_left=V1*0.05-WheelD1;
		error_right=V2*0.05-WheelD2;
		error3=V3*0.05-WheelD3;
	
		integral_left = integral_left + error_left;
		integral_right = integral_right + error_right;
		integral_right = integral3 + error3;
	
		derivative_left=error_left-errorp1;
		derivative_right=error_right-errorp2;
		derivative3=error3-errorp3;

		duty_left=-1*PID_duty(error_left, kp, ki, kd, integral_left, derivative_left);
		duty_right=-1*PID_duty(error_right, kp_M2, ki_M2, kd_M2, integral_right, derivative_right);
		M3_Duty=-1*PID_duty(error3, kp_M3, ki_M3, kd_M3, integral3, derivative3);

		// soft start part
		if(duty_left-duty_left_p>0.3)
	  		duty_left = duty_left_p+0.3;
		else if( duty_left-duty_left_p<-0.3 )
	  		duty_left = duty_left_p-0.3;

		if(duty_right-duty_right_p>0.3)
	  		duty_right = duty_right_p+0.3;
		else if( duty_right-duty_right_p<-0.3 )
	  		duty_right = duty_right_p-0.3;

		if(M3_Duty-M3_Duty_p>0.3)
	 		M3_Duty = M3_Duty_p+0.3;
		else if( duty_right-duty_right_p<-0.3 )
	  		M3_Duty = M3_Duty_p-0.3;

		duty_left_p=duty_left;
		duty_right_p=duty_right;
		M3_Duty_p=M3_Duty;

		rc_set_motor(Channel_Left, duty_left);
		rc_set_motor(Channel_Right, duty_right);
		rc_set_motor(Motor3Channel, M3_Duty);
		//double secs =ros::Time::now().toSec();
		//ROS_INFO("time= %0.8f ", secs);
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("Vf1, Vf2, Vf3= %0.6f %0.6f %0.6f", WheelD1/0.05, WheelD2/0.05, WheelD3/0.05);
	ROS_INFO("D1, D2, D3= %1.2f %1.2f %1.2f", duty_left, duty_right, M3_Duty);
	
	rc_set_encoder_pos(Channel_Left,0);
  	rc_set_encoder_pos(Channel_Right,0);
	rc_set_encoder_pos(Motor3Channel,0);

//	rc_usleep(10000);

}

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
//  ros::spin();
  return 0;

  rc_set_motor(Channel_Left,0);
  rc_set_motor(Channel_Right,0);
  rc_set_motor(Motor3Channel,0);

}

