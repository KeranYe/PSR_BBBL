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
#include <psr_smach/PSR_Drive.h>
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

// Sampling
int num_sample = 10;

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


void drive_Callback(const psr_smach::PSR_Drive::ConstPtr& psr_drive_msg)
{

	// Here we borrow geometry_msgs/Twist to store desired states: positions and velocities.
  	// Will create specific message type for PSR later.


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
		cout << "Please enter left motor direction (1 or -1, float): ";
		cin >> motor_left_dir;
		cout << "Please enter right motor direction (1 or -1, float): ";
		cin >> motor_right_dir;
		cout << "Please enter soft portion for duty cycle ((0,0.3], float): ";
		cin >> duty_soft;
		cout << "Please enter sampling number (int): ";
		cin >> num_sample;
		
		cout << "Correct input for" << endl; 
		cout << "motor_left_dir = " << motor_left_dir << endl;
		cout << "motor_right_dir = " << motor_right_dir << endl;
		cout << "duty_soft = " << duty_soft << endl;
		cout << "num_sample = " << num_sample << "?(y/n)";
		cin >> yes_or_no;		
		if(yes_or_no == 'y') break;
	}

	// 1.2 Temp states
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
	
	ros::init(argc, argv, "psr_drive");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/PSR/motors", 10, drive_Callback); // Need a reset flag in self-defined msg

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
	ros::Rate r(500);  //500 hz
	// Assume initial pos for encoder has been set by funtion rc_test_encoders
	//int num_sample = 10; 
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
		// 2.1 Compute average position and velocity in 5 samples
		int tick_left_sum = 0;
		int tick_left_0 = 0;
		int tick_right_sum = 0;
		int tick_right_0 = 0;
		for (int i = 0; i < num_sample; ++i){
			// 2.1.1 Get position
			if(i == 0){
				tick_left_0 = (-1)*rc_get_encoder_pos(Channel_Left); // Positive number
				tick_right_0 = (1)*rc_get_encoder_pos(Channel_Right); // Positive number
				tick_left_sum += tick_left_0; // Positive number
				tick_right_sum += tick_right_0;} // Positive number
			else{
				tick_left_sum += (-1)*rc_get_encoder_pos(Channel_Left);
				tick_right_sum += (1)*rc_get_encoder_pos(Channel_Right);}
			
			// Reset encoder if necessary		
			if(i == (num_sample-1) && (-1)*rc_get_encoder_pos(Channel_Left) > total_tick)
				rc_set_encoder_pos(Channel_Left, (-1)*rc_get_encoder_pos(Channel_Left) - total_tick);
				 //rc_set_encoder_pos(Channel_Left, rc_get_encoder_pos(Channel_Left) + total_tick); // negative offset
			if(i == (num_sample-1) && (1)*rc_get_encoder_pos(Channel_Right) > total_tick)
				rc_set_encoder_pos(Channel_Right, (1)*rc_get_encoder_pos(Channel_Right) - total_tick); // positive offset
		
			ros::spinOnce();
			r.sleep();
		}

		// 2.2 Compute average position and speed
		int tick_left_average = tick_left_sum / num_sample % total_tick;
		int tick_right_average = tick_right_sum / num_sample % total_tick;
		
		if(tick_left_average < 0)
			tick_left_average += total_tick;
		if(tick_right_average < 0)
			tick_right_average += total_tick;
			
		int tick_left_diff = tick_left_sum - tick_left_0;
		int tick_right_diff = tick_right_sum - tick_right_0;
		// 2.2.1 Compute position
		pos_left = ((float)tick_left_average * TWO_PI) / ((float)total_tick);
		pos_right = ((float)tick_right_average * TWO_PI) / ((float)total_tick);
		pos_left_diff = pos_left - pos_left_prior;
		pos_right_diff = pos_right - pos_right_prior;
		
		// Special case when pos near 0
		if(pos_left_diff < ((-1)*PI))
			pos_left_diff += TWO_PI;
		else if(pos_left_diff > PI)
			pos_left_diff -= TWO_PI;		
		else
			pos_left_diff = pos_left_diff;
		
		// Special case when pos near 0
		if(pos_right_diff < ((-1)*PI))
			pos_right_diff += TWO_PI;
		else if(pos_right_diff > PI)
			pos_right_diff -= TWO_PI;		
		else
			pos_right_diff = pos_right_diff;
		
		// 2.2.2 Compute velocity	
		vel_left = pos_left_diff / (0.002*(float)num_sample); // rad per second
		vel_right = pos_right_diff / (0.002*(float)num_sample); // rad per second
		//vel_left = ((float)tick_left_diff * TWO_PI) / (((float)total_tick) * (0.002*(float)(num_sample-1)));
		vel_left_diff = vel_left - vel_left_prior;
		vel_right_diff = vel_right - vel_right_prior;
		// 2.2.3 Compute acceleration
		accel_left = vel_left_diff / (0.002*(float)num_sample); // rad per second square
		accel_right = vel_right_diff / (0.002*(float)num_sample); // rad per second square
			
		// Publish position and velocity
		ROS_INFO("pos_left = %0.6f, pos_right = %0.6f, vel_left = %0.6f, vel_right = %0.6f", \
			 pos_left * 360 / TWO_PI, pos_right * 360 / TWO_PI, vel_left * 360 / TWO_PI, vel_right * 360 / TWO_PI);
		
		
		// 2.3 PID controller for duty
		// This is a velocity PID controller, where duty = kp * (vel_des - vel) + ki * sum(vel_des - vel) + kd * (accel_des - accel)
		
		// 2.3.1 errors
		error_left = vel_left_des - vel_left;
		error_right = vel_right_des - vel_right;
		//error_left = pos_left_des - pos_left;
		//error_right = pos_right_des - pos_right;
/*		
		// Special case when pos near 0
		if(error_left > PI)
			error_left = error_left - TWO_PI;
		else if(error_left < ((-1)*PI))
			error_left = error_left + TWO_PI;
		else
			error_left = error_left;
		
		// Special case when pos near 0	
		if(error_right > PI)
			error_right = error_right - TWO_PI;
		else if(error_right < ((-1)*PI))
			error_right = error_right + TWO_PI;
		else
			error_right = error_right;
*/		
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
		
		duty_left_prior = duty_left;
		duty_right_prior = duty_right;
	
		// Set motor duty
		rc_set_motor(Channel_Left, duty_left);
		rc_set_motor(Channel_Right, duty_right);

		// Publish duty
		//ROS_INFO("duty_left = %0.2f, duty_right = %0.2f", duty_left, duty_right);
		
		pos_left_prior = pos_left;
		pos_right_prior = pos_right;
		vel_left_prior = vel_left;
		vel_right_prior = vel_right;
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

