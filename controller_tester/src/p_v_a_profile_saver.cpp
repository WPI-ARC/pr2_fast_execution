#include "ros/ros.h"
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <cmath> 
#include <time.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int count=0;
float sum_p[7] = { 0, 0, 0, 0, 0, 0, 0 };
float sum_v[7] = { 0, 0, 0, 0, 0, 0, 0 };
bool countstart=false;
float ave_p[7] = { 0, 0, 0, 0, 0, 0, 0 };
float ave_v[7] = { 0, 0, 0, 0, 0, 0, 0 };
bool countChange=false;

float p_desire_1[50000];
float v_desire_1[50000];
float a_desire_1[50000];
float p_desire_2[50000];
float v_desire_2[50000];
float a_desire_2[50000];
float p_desire_3[50000];
float v_desire_3[50000];
float a_desire_3[50000];
float p_desire_4[50000];
float v_desire_4[50000];
float a_desire_4[50000];
float p_desire_5[50000];
float v_desire_5[50000];
float a_desire_5[50000];
float p_desire_6[50000];
float v_desire_6[50000];
float a_desire_6[50000];
float p_desire_7[50000];
float v_desire_7[50000];
float a_desire_7[50000];


float p_actual_1[50000];
float v_actual_1[50000];


float p_actual_2[50000];
float v_actual_2[50000];


float p_actual_3[50000];
float v_actual_3[50000];


float p_actual_4[50000];
float v_actual_4[50000];


float p_actual_5[50000];
float v_actual_5[50000];


float p_actual_6[50000];
float v_actual_6[50000];


float p_actual_7[50000];
float v_actual_7[50000];


float p_error_1[50000];
float p_error_2[50000];
float p_error_3[50000];
float p_error_4[50000];
float p_error_5[50000];
float p_error_6[50000];
float p_error_7[50000];

float v_error_1[50000];
float v_error_2[50000];
float v_error_3[50000];
float v_error_4[50000];
float v_error_5[50000];
float v_error_6[50000];
float v_error_7[50000];


bool startCount=false;
bool finishCount=false;
int printTimes=0;
double startTime;
double finishTime;
double duration;

//callback function when receive message from controller state
void chatterCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
	//start to gather data
	if (startCount==true){
    p_desire_1[count]=msg->desired.positions[0];
		p_desire_2[count]=msg->desired.positions[1];
		p_desire_3[count]=msg->desired.positions[2];
		p_desire_4[count]=msg->desired.positions[3];
		p_desire_5[count]=msg->desired.positions[4];
		p_desire_6[count]=msg->desired.positions[5];
		p_desire_7[count]=msg->desired.positions[6];

    v_desire_1[count]=msg->desired.velocities[0];
		v_desire_2[count]=msg->desired.velocities[1];
		v_desire_3[count]=msg->desired.velocities[2];
		v_desire_4[count]=msg->desired.velocities[3];
		v_desire_5[count]=msg->desired.velocities[4];
		v_desire_6[count]=msg->desired.velocities[5];
		v_desire_7[count]=msg->desired.velocities[6];

    a_desire_1[count]=msg->desired.accelerations[0];


    p_actual_1[count]=msg->actual.positions[0];
    v_actual_1[count]=msg->actual.velocities[0];
    p_actual_2[count]=msg->actual.positions[1];
    v_actual_2[count]=msg->actual.velocities[1];
    p_actual_3[count]=msg->actual.positions[2];
    v_actual_3[count]=msg->actual.velocities[2];
    p_actual_4[count]=msg->actual.positions[3];
    v_actual_4[count]=msg->actual.velocities[3];
    p_actual_5[count]=msg->actual.positions[4];
    v_actual_5[count]=msg->actual.velocities[4];
    p_actual_6[count]=msg->actual.positions[5];
    v_actual_6[count]=msg->actual.velocities[5];
    p_actual_7[count]=msg->actual.positions[6];
    v_actual_7[count]=msg->actual.velocities[6];
    
    p_error_1[count]=msg->error.positions[0];
    p_error_2[count]=msg->error.positions[1];
    p_error_3[count]=msg->error.positions[2];
    p_error_4[count]=msg->error.positions[3];
    p_error_5[count]=msg->error.positions[4];
    p_error_6[count]=msg->error.positions[5];
    p_error_7[count]=msg->error.positions[6];



    v_error_1[count]=msg->error.velocities[0];
    v_error_2[count]=msg->error.velocities[1];
    v_error_3[count]=msg->error.velocities[2];
    v_error_4[count]=msg->error.velocities[3];
    v_error_5[count]=msg->error.velocities[4];
    v_error_6[count]=msg->error.velocities[5];
    v_error_7[count]=msg->error.velocities[6];


		for (int i = 0; i<7; i++)
		{
			sum_p[i] = sum_p[i] + msg->error.positions[i];
			sum_v[i] = sum_v[i] + msg->error.velocities[i];
		}

		count++;
		
	}

	//stop gather data and print data
	if (finishCount==true){

		for (int i = 0; i<7; i++)
		{
			ave_p[i] = sum_p[i]/count;
			ave_v[i] = sum_v[i]/count;
		}

		ROS_INFO("position error: [%f, %f, %f, %f, %f, %f, %f]", ave_p[0], ave_p[1], ave_p[2], ave_p[3], ave_p[4], ave_p[5], ave_p[6]);
		ROS_INFO("velocity error: [%f, %f, %f, %f, %f, %f, %f]", ave_v[0], ave_v[1], ave_v[2], ave_v[3], ave_v[4], ave_v[5], ave_v[6]);

		ROS_INFO("count: %d", count);
                printf("---------------------------------------------------desire_position_1--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",p_desire_1[i]);
                }
                
                printf("---------------------------------------------------desire_position_2--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",p_desire_2[i]);
                }
                
                printf("---------------------------------------------------desire_position_3--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",p_desire_3[i]);
                }
                printf("---------------------------------------------------desire_position_4--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",p_desire_4[i]);
                }
                printf("---------------------------------------------------desire_position_5--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",p_desire_5[i]);
                }
                
                printf("---------------------------------------------------desire_position_6--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",p_desire_6[i]);
                }
                
                printf("---------------------------------------------------desire_position_7--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",p_desire_7[i]);
                }
								
                printf("---------------------------------------------------desire_velocity_1--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",v_desire_1[i]);
                }
                printf("---------------------------------------------------desire_velocity_2--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",v_desire_2[i]);
                }
                printf("---------------------------------------------------desire_velocity_3--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",v_desire_3[i]);
                }
                printf("---------------------------------------------------desire_velocity_4--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",v_desire_4[i]);
                }
                printf("---------------------------------------------------desire_velocity_5--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",v_desire_5[i]);
                }
								
                printf("---------------------------------------------------desire_velocity_6--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",v_desire_6[i]);
                }
                printf("---------------------------------------------------desire_velocity_7--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",v_desire_7[i]);
                }

                
                
                printf("---------------------------------------------------actual_position_1--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", p_actual_1[i]);
                }
                printf("---------------------------------------------------actual_position_2--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", p_actual_2[i]);
                }
                printf("---------------------------------------------------actual_position_3--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", p_actual_3[i]);
                }
                printf("---------------------------------------------------actual_position_4--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", p_actual_4[i]);
                }
                printf("---------------------------------------------------actual_position_5--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", p_actual_5[i]);
                }
                printf("---------------------------------------------------actual_position_6--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", p_actual_6[i]);
                }
                printf("---------------------------------------------------actual_position_7--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", p_actual_7[i]);
                }


                printf("---------------------------------------------------actual_velocity_1--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", v_actual_1[i]);
                }
                printf("---------------------------------------------------actual_velocity_2--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", v_actual_2[i]);
                }
                printf("---------------------------------------------------actual_velocity_3--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", v_actual_3[i]);
                }
                printf("---------------------------------------------------actual_velocity_4--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", v_actual_4[i]);
                }
                printf("---------------------------------------------------actual_velocity_5--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", v_actual_5[i]);
                }
                printf("---------------------------------------------------actual_velocity_6--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", v_actual_6[i]);
                }
                printf("---------------------------------------------------actual_velocity_7--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n", v_actual_7[i]);
                }



                printf("---------------------------------------------------desire_acceleration_1--------------------------------------------------------------\n");
                for (int i=0; i<count; i++){
                  printf("%f \n",a_desire_1[i]);
                }


		printTimes++;
		count=0;
		startCount=false;
		finishCount=false;
                for (int i=0; i<7; i++)		
		{
			sum_p[i] = 0;
		}
                for (int j=0; j<7; j++)		
		{
			sum_v[j] = 0;
		}

	}
	
	
}

//finish gather data when trajectory execution is finished
void controllerResultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg){
	ROS_INFO("resultCB");
	finishCount=true;
	finishTime=(double)msg->header.stamp.toSec();
}

//start gather data when receive a goal
void controllerGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg){
	ROS_INFO("goalCB");
	
	startCount=true;
	startTime=(double)msg->header.stamp.toSec();
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "pva");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/r_arm_controller/state", 10000, chatterCallback);
	ros::Subscriber sub1 = n.subscribe("/r_arm_controller/follow_joint_trajectory/goal", 1000, controllerGoalCallback);
	ros::Subscriber sub2 = n.subscribe("/r_arm_controller/follow_joint_trajectory/result", 1000, controllerResultCallback);

  ros::spin();

  return 0;
}
