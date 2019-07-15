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

// Define Globle Variables
float V1_max = 3.5;
float V2_max = 3.5;
float V3_max = 3.5; 
float V1 = 0; 
float V2 = 0; 
float V3 = 0;

// Define duty cycle for dual motors

//motors
unsigned int Motor1Channel = 1;
unsigned int Motor2Channel = 2;
unsigned int Motor3Channel = 3;
float gearbox = 51.45;
int encoder_res=12;

//PID
float kp_M1 = 10;
float ki_M1 = 0;
float kd_M1 = 15; 

float kp_M2 = 10;
float ki_M2 = 0;
float kd_M2 = 15;
  
float kp_M3 = 10;
float ki_M3 = 0;
float kd_M3 = 15;

//duty cycle
float M1_Duty = 0;
float M2_Duty = 0;
float M3_Duty = 0; 

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void drive_Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel_twist)
{

// assign the commands if the array is of the correct length


//formulas for velocity 
  float Vx =0;
  float Vy =0; 
  float Wz=0;
  float L= 0.5;
  Vx= cmd_vel_twist -> linear.x;
  Vy= cmd_vel_twist -> linear.y;
  Wz = cmd_vel_twist -> angular.z;


//------------kinematics ----------


  V1= -(Vx)/2 - (sqrt(3)*Vy)/2 + L*Wz;
  V2= Vx + L*Wz;
  V3= -(Vx)/2+sqrt(3)*Vy/2+ L*Wz;

   if( M1_Duty > V1_max ){
   	M1_Duty = 0;
	ROS_INFO("Motor 1 upper bound excessed!");
   }
   if( M2_Duty > V2_max ){
        M2_Duty = 0;
	ROS_INFO("Motor 2 upper bound excessed!");
   }

  if( M3_Duty > V3_max ){
        M3_Duty = 0;
	ROS_INFO("Motor 3 upper bound excessed!");
   }

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

float PID_wheel(float error, float kp, float ki, float kd, float integral, float derivative)
{

    // Proportional term
    float Pout = kp * error;

    // Integral term
    float Iout = ki * integral;

    // Derivative term
    float Dout = kd * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > 1 )
        output = 1;
    else if( output < -1 )
        output = -1;

return output;
}


int main(int argc, char **argv)
{
  float WheelD1=0;
  float WheelD2=0;
  float WheelD3=0;

  float error1=0;
  float error2=0;
  float error3=0;

  float integral1=0;
  float integral2=0;
  float integral3=0;
	
  float derivative1=0;
  float derivative2=0;
  float derivative3=0;

  float errorp1=0;
  float errorp2=0;
  float errorp3=0;

  float M1_Duty_p=0;
  float M2_Duty_p=0;
  float M3_Duty_p=0;
	
  ros::init(argc, argv, "omni_drive");

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
  ros::Rate r(500);  //500 hz
  while(ros::ok()){
	//rc_enable_motors();
	for (int i = 0; i < 25; ++i){

	WheelD1 = (rc_get_encoder_pos(Motor1Channel) * TWO_PI)/(1 * gearbox * encoder_res)*0.06;
	WheelD2 = (rc_get_encoder_pos(Motor2Channel) * TWO_PI)/(1 * gearbox * encoder_res)*0.06;
	WheelD3 = (rc_get_encoder_pos(Motor3Channel) * TWO_PI)/(1 * gearbox * encoder_res)*0.06;
	
	error1=V1*0.05-WheelD1;
	error2=V2*0.05-WheelD2;
	error3=V3*0.05-WheelD3;
	
	integral1 = integral1 + error1;
	integral2 = integral2 + error2;
	integral2 = integral3 + error3;
	
	derivative1=error1-errorp1;
	derivative2=error2-errorp2;
	derivative3=error3-errorp3;

	M1_Duty=-1*PID_wheel(error1, kp_M1, ki_M1, kd_M1, integral1, derivative1);
	M2_Duty=-1*PID_wheel(error2, kp_M2, ki_M2, kd_M2, integral2, derivative2);
	M3_Duty=-1*PID_wheel(error3, kp_M3, ki_M3, kd_M3, integral3, derivative3);

	// soft start part
	if(M1_Duty-M1_Duty_p>0.3)
	  M1_Duty = M1_Duty_p+0.3;
	else if( M1_Duty-M1_Duty_p<-0.3 )
	  M1_Duty = M1_Duty_p-0.3;

	if(M2_Duty-M2_Duty_p>0.3)
	  M2_Duty = M2_Duty_p+0.3;
	else if( M2_Duty-M2_Duty_p<-0.3 )
	  M2_Duty = M2_Duty_p-0.3;

	if(M3_Duty-M3_Duty_p>0.3)
	  M3_Duty = M3_Duty_p+0.3;
	else if( M2_Duty-M2_Duty_p<-0.3 )
	  M3_Duty = M3_Duty_p-0.3;

	M1_Duty_p=M1_Duty;
	M2_Duty_p=M2_Duty;
	M3_Duty_p=M3_Duty;

	rc_set_motor(Motor1Channel, M1_Duty);
	rc_set_motor(Motor2Channel, M2_Duty);
	rc_set_motor(Motor3Channel, M3_Duty);
	//double secs =ros::Time::now().toSec();
	//ROS_INFO("time= %0.8f ", secs);
	ros::spinOnce();
	r.sleep();
	}
	ROS_INFO("Vf1, Vf2, Vf3= %0.6f %0.6f %0.6f", WheelD1/0.05, WheelD2/0.05, WheelD3/0.05);
	ROS_INFO("D1, D2, D3= %1.2f %1.2f %1.2f", M1_Duty, M2_Duty, M3_Duty);
	
	rc_set_encoder_pos(Motor1Channel,0);
  	rc_set_encoder_pos(Motor2Channel,0);
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

  rc_set_motor(Motor1Channel,0);
  rc_set_motor(Motor2Channel,0);
  rc_set_motor(Motor3Channel,0);

}

