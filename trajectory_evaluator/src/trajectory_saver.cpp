//trajectory saver will listen on topic which a trajectory is published and extract that trajectory and export it as xtf file.
#include "trajectory_evaluator/xtf.h"
#include <exception>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <string>

std::string file_name;


//subscribe to gather trajectory information
void goalCB(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("Got Trajectory");
  std::string uid="AA";
	XTF::Trajectory::TRAJTYPES traj_type=XTF::Trajectory::GENERATED;
	XTF::Trajectory::TIMINGS timing;
	XTF::Trajectory::DATATYPES data_type=XTF::Trajectory::JOINT;
  std::string robot="pr2";
	std::string generator="reference";
  std::vector<std::string> joint_names;
	std::vector<XTF::State> trajectory_data;
	std::vector<std::string> tags;

  for (unsigned int ind=0; ind<msg->goal.trajectory.points.size(); ind++)
	{

    std::vector< double > desiredPosition;
    std::vector< double > desiredVelocity;
    std::vector< double > desiredAcceleration;
    std::vector< double > actualPosition;
    std::vector< double > actualVelocity;
    std::vector< double > actualAcceleration;
    timespec timing;
    int sequence=ind;
		for (int i=0; i<7; i++)
		{

 			desiredPosition.push_back((double)(msg->goal.trajectory.points[ind].positions[i]));

 			desiredVelocity.push_back((double)(msg->goal.trajectory.points[ind].velocities[i]));
 			desiredAcceleration.push_back((double)(msg->goal.trajectory.points[ind].accelerations[i]));
		}

    timing.tv_sec=msg->goal.trajectory.points[ind].time_from_start.sec;
    timing.tv_nsec=msg->goal.trajectory.points[ind].time_from_start.nsec;
    XTF::State new_state(desiredPosition, desiredVelocity, desiredAcceleration, actualPosition, actualVelocity, actualAcceleration, sequence, timing);
    trajectory_data.push_back(new_state);
	}

  XTF::Trajectory my_traj=XTF::Trajectory(uid, traj_type, timing, data_type, robot, generator, joint_names, trajectory_data, tags);
  XTF::Parser parser;

	//save trajectory as xtf file
	parser.ExportTraj(my_traj, file_name, false);	
		

  ROS_INFO("Done");
}

int main(int argc, char **argv)
{
   
  std::cout << "Enter the name of file you want to save as: " << std::endl;
  getline (std::cin, file_name);


  ros::init(argc, argv, "trajectory_saver");

  
  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("/r_arm_controller/follow_joint_trajectory/goal", 1000, goalCB);

  
  ros::spin();

  return 0;
}
