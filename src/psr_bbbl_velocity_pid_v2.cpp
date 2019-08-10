extern "C"
{
#include "rc_usefulincludes.h"
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
#include <iostream>
#include <psr_msgs/PSR_Drive.h>
using namespace std;

//-----------------------------
//------ Global Variables------
//-----------------------------

// Flag for general purpose
char yes_or_no = 'n';
bool reset = false;
//-----------------------------
// Define States: 
// pos = omega, vel = omega_dot
//-----------------------------

// State limitation
float vel_left_max = 3.5;
float vel_right_max = 3.5;

// Real states: [0, 2*pi)
float pos_left = 0; 
float pos_right = 0; 

float vel_left = 0; 
float vel_right = 0; 

float accel_left = 0;
float accel_right = 0;

// Desired states 
float pos_left_des = 0; 
float pos_right_des = 0; 

float vel_left_des = 0; 
float vel_right_des = 0; 

float accel_left_des = 0;
float accel_right_des = 0;

//-----------------------------
// Define Motor Parameters
//-----------------------------

// Motor channel
unsigned int Channel_Left = 1;
unsigned int Channel_Right = 2;

// Moter mechanical property
float gear_ratio = 297.9;
float encoder_res = 12.0;
int total_tick = (int)(gear_ratio * encoder_res);

// Motor duty cycle
float duty_left = 0;
float duty_right = 0; 
float duty_soft = 0;

// Motor direction
float motor_left_dir = -1.0;
float motor_right_dir = 1.0;

// Wheel direction
float wheel_dir_left = 1.0;
float wheel_dir_right = -1.0;

// Sampling
int num_sample = 3;
unsigned int rate = 500;

//-----------------------------
// Define Controller Parameters
//-----------------------------

//PID Coeffients
float kp = 1;
float ki = 0;
float kd = 0; 


//-----------------------------
//------ Functions-------------
//-----------------------------

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}


void drive_Callback(const psr_msgs::PSR_Drive::ConstPtr& psr_drive_msg)
{

	// Here we borrow geometry_msgs/Twist to store desired states: positions and velocities.
  	// Will create specific message type for PSR later.

	ROS_INFO("Message received!");
  	//Retrieve data to global state variables
  	reset = psr_drive_msg -> reset;
	pos_left_des = psr_drive_msg -> theta_left_des;
  	pos_right_des = psr_drive_msg -> theta_right_des;
	vel_left_des = psr_drive_msg -> omega_left_des;
  	vel_right_des = psr_drive_msg -> omega_right_des;
	accel_left_des = psr_drive_msg -> alpha_left_des;
	accel_right_des = psr_drive_msg -> alpha_right_des;

}


void ros_compatible_shutdown_signal_handler(int signo)
{
	if (signo == SIGINT)
	{
		rc_set_motor(Channel_Left,0);
  		rc_set_motor(Channel_Right,0);
		rc_disable_motors();		
		rc_set_state(EXITING);
		ROS_INFO("\nReceived SIGINT Ctrl-C.");
		ros::shutdown();
	}
	else if (signo == SIGTERM)
	{
		rc_set_motor(Channel_Left,0);
  		rc_set_motor(Channel_Right,0);
		rc_disable_motors();		
		rc_set_state(EXITING);
		ROS_INFO("Received SIGTERM.");
		ros::shutdown();
	}
}


float PID_duty(float error, float integral, float derivative)
{
	// This PID aims to achieve position control.
	float duty = kp * error + ki * integral + kd * derivative; 

	// Limit check
	if( duty > 1 )
		duty = 1;
	else if( duty < -1 )
		duty = -1;
	else
		return duty;

	return duty;
}

float getPartialAverage(float arr[], int a, int b) {
	int i;
  	float sum = 0;       
  	float avg;          

   for (i = a; i < (b+1); ++i) {
      sum += arr[i];
   }
   avg = sum / (b-a+1);

   return avg;
}

int main(int argc, char **argv)
{
	//------------------	
	// 1. Initialzation
	//------------------	

	// 1.1 Input from command line
	
	// PID parameters
	while(1){	
		yes_or_no = 'n';		
		cout << "Please enter kp (float): ";
		cin >> kp;
		cout << "Please enter ki (float): ";
		cin >> ki;
		cout << "Please enter kd (float): ";
		cin >> kd;
		cout << "Correct input for kp = " << kp << " and ki = " << ki << " and kd = " << kd << "?(y/n)";
		cin >> yes_or_no;		
		if(yes_or_no == 'y') break;
	}
	
	// Motor properties
	while(1){
		yes_or_no = 'n';
		cout << "Please enter left motor direction (1 or -1, float, default = -1): ";
		cin >> motor_left_dir;
		cout << "Please enter right motor direction (1 or -1, float, default = 1): ";
		cin >> motor_right_dir;
		cout << "Please enter soft portion for duty cycle ((0,0.3], float): ";
		cin >> duty_soft;
		cout << "Please enter sampling number (int, odd, default = 3): ";
		cin >> num_sample;
		cout << "Please enter sampling rate (int, default = 500): ";
		cin >> num_sample;
		
		cout << "Correct input for" << endl; 
		cout << "motor_left_dir = " << motor_left_dir << endl;
		cout << "motor_right_dir = " << motor_right_dir << endl;
		cout << "duty_soft = " << duty_soft << endl;
		cout << "rate = " << rate << endl;
		cout << "num_sample = " << num_sample << "?(y/n)";
		cin >> yes_or_no;		
		if(yes_or_no == 'y') break;
	}

	// 1.2 Temp states
	float pos_array_left[3*num_sample] = {};
	float pos_array_right[3*num_sample] = {};
	
	float error_left = 0;
	float error_right = 0;

	float integral_left = 0;
	float integral_right = 0;
	
	float derivative_left = 0;
	float derivative_right = 0;

	float error_left_prior = 0;
	float error_right_prior = 0;
	
	float pos_left_prior = 0;
	float pos_right_prior = 0;

	float vel_left_prior = 0;
	float vel_right_prior = 0;

	float pos_left_diff = 0;
	float pos_right_diff = 0;

	float vel_left_diff = 0;
	float vel_right_diff = 0;

	float duty_left_prior = 0;
	float duty_right_prior = 0;
	
	// 1.3 Node parameter
	unsigned int call_back_queue_len = 100;
	
	ros::init(argc, argv, "psr_drive");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/PSR/motors", call_back_queue_len, drive_Callback); // Need a reset flag in self-defined msg

	if(rc_initialize()<0)
	{
		ROS_INFO("ERROR: failed to initialize cape.");
		return -1;
	}

	signal(SIGINT,  ros_compatible_shutdown_signal_handler);	
	signal(SIGTERM, ros_compatible_shutdown_signal_handler);


	//------------------	
	// 2. Algorithm
	//------------------

	rc_enable_motors();
	ros::Rate r(rate);  //500 hz
	// Assume initial pos for encoder has been set by funtion rc_test_encoders
	//int num_sample = 25; 
	int first_index = num_sample-1;
	int second_index = 2*num_sample-1;
	int third_index = 3*num_sample-1;
	while(ros::ok()){
		// Check reset flag
		if(reset){
			// reset pid terms
			error_left = 0;
			error_right = 0;

			integral_left = 0;
			integral_right = 0;
	
			derivative_left = 0;
			derivative_right = 0;
			
			reset = !reset;
		}
		// Reset encoder
		rc_set_encoder_pos(Channel_Left,0);
		rc_set_encoder_pos(Channel_Right,0);
		
		// 2.1 Compute and store position in 25 samples
		for (int i = 0; i < (3*num_sample); ++i){
			pos_array_left[i] = wheel_dir_left * (float)rc_get_encoder_pos(Channel_Left) * TWO_PI / (float)total_tick;
			pos_array_right[i] = wheel_dir_right * (float)rc_get_encoder_pos(Channel_Right) * TWO_PI / (float)total_tick;
			
//			ros::spinOnce();
			r.sleep();
		}
														     
		// 2.2 Compute velocity and acceleration for middle sample
		// get position for 3 samples
		float pos_1_left = getPartialAverage(pos_array_left, 0, first_index);
		float pos_2_left = getPartialAverage(pos_array_left, num_sample, second_index);
		float pos_3_left = getPartialAverage(pos_array_left, 2*num_sample, third_index);
		float pos_1_right = getPartialAverage(pos_array_right, 0, first_index);
		float pos_2_right = getPartialAverage(pos_array_right, num_sample, second_index);
		float pos_3_right = getPartialAverage(pos_array_right, 2*num_sample, third_index);
		// 1/4 and 3/4 point velocity
		float vel_half_1_left = (pos_2_left - pos_1_left) / ((float)first_index * 0.002);
		float vel_half_2_left = (pos_3_left - pos_2_left) / ((float)first_index * 0.002);												    
		float vel_half_1_right = (pos_2_right - pos_1_right) / ((float)first_index * 0.002);
		float vel_half_2_right = (pos_3_right - pos_2_right) / ((float)first_index * 0.002);												    
		// middle point velocity and acceleration												     
		vel_left = (vel_half_2_left + vel_half_1_left) / 2.0;
		vel_right = (vel_half_2_right + vel_half_1_right) / 2.0;
		accel_left = (vel_half_2_left - vel_half_1_left) / ((float)first_index * 0.002);
		accel_right = (vel_half_2_right - vel_half_1_right) / ((float)first_index * 0.002);
			
		// Publish velocity
		ROS_INFO("vel_left = %f, vel_right = %f", \
			 vel_left, vel_right);
//		ROS_INFO("vel_left = %0.6f, vel_right = %0.6f", \
//			 vel_left * 360 / TWO_PI, vel_right * 360 / TWO_PI);
				
		// 2.3 PID controller for duty
		// This is a velocity PID controller, 
		// where duty = kp * (vel_des - vel) + ki * sum(vel_des - vel) + kd * (accel_des - accel)
		
		// 2.3.1 errors
		error_left = vel_left_des - vel_left;
		error_right = vel_right_des - vel_right;
		//error_left = pos_left_des - pos_left;
		//error_right = pos_right_des - pos_right;
														     
		// 2.3.2 derivatives
		derivative_left = accel_left_des - accel_left;
		derivative_right = accel_right_des - accel_right;
		//derivative_left = vel_left_des - vel_left;
		//derivative_right = vel_right_des - vel_right;
		
		// 2.3.3 integrals
		integral_left = integral_left + error_left;
		integral_right = integral_right + error_right;
		//integral_left = 0.0;
		//integral_right = 0.0;
		
		// 2.3.4 duty from PID
		duty_left = motor_left_dir*PID_duty(error_left, integral_left, derivative_left);
		duty_right = motor_right_dir*PID_duty(error_right, integral_right, derivative_right);

		// Soft start
		if((duty_left - duty_left_prior) > duty_soft)
	  		duty_left = duty_left_prior + duty_soft;
		else if((duty_left - duty_left_prior) < - duty_soft )
	  		duty_left = duty_left_prior - duty_soft;
		
		if((duty_right - duty_right_prior) > duty_soft)
	  		duty_right = duty_right_prior + duty_soft;
		else if((duty_right - duty_right_prior) < - duty_soft )
	  		duty_right = duty_right_prior - duty_soft;
		
		
	
		// Set motor duty
		rc_set_motor(Channel_Left, duty_left);
		rc_set_motor(Channel_Right, duty_right);

		// Publish duty
		ROS_INFO("duty_left = %0.2f, duty_right = %0.2f", duty_left, duty_right);
														     
		duty_left_prior = duty_left;
		duty_right_prior = duty_right;
		
		ros::spinOnce();
//		r.sleep();
		//rc_set_motor(Channel_Left, (-1)*(0.3));// Velocity Test
	}

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
//  ros::spin();
	return 0;

}
