//send trajectory to controller with input of a xtf file
#include "trajectory_evaluator/xtf.h"
#include <exception>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;
int joint_map [7] = { 3, 2, 4, 0, 1, 5, 6 }; 

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/r_arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(XTF::Trajectory my_traj)
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    int selection;
    std::cout << "Please enter your selection: " << std::endl;
    std::cout << "0: go default" << std::endl;
    std::cout << "1: go desire" << std::endl;
    std::cout << "2: get another trajectory file" << std::endl;
    std::cin >> selection;

    if (selection==1)
    {
      int trajectory_size=my_traj.trajectory.size();
      goal.trajectory.points.resize(trajectory_size);


      for (int ind=0; ind<trajectory_size; ind++)
      {
        // Positions
        goal.trajectory.points[ind].positions.resize(7);
        
      
        for (int i=0; i<7; i++)
        {
          
          goal.trajectory.points[ind].positions[i]=my_traj.trajectory[ind].position_desired[joint_map[i]];
        }
        // Velocities
        goal.trajectory.points[ind].velocities.resize(7);
        
        for (int i=0; i<7; i++)
        {
          goal.trajectory.points[ind].velocities[i]=my_traj.trajectory[ind].velocity_desired[joint_map[i]];
        }
        // Accelerations
        goal.trajectory.points[ind].accelerations.resize(7);
        
        for (int i=0; i<7; i++)
        {
          goal.trajectory.points[ind].accelerations[i]=my_traj.trajectory[ind].acceleration_desired[joint_map[i]];
        }
        goal.trajectory.points[ind].time_from_start = ros::Duration(my_traj.trajectory[ind].timing.tv_sec, my_traj.trajectory[ind].timing.tv_nsec);
      
      }

      printf("---------------------------------------------------Position--------------------------------------------------------------");
      for (int i=0; i<trajectory_size; i++){
        printf("%f \n",goal.trajectory.points[i].positions[5]);
      }

      printf("---------------------------------------------------Velocity--------------------------------------------------------------");
      for (int i=0; i<trajectory_size; i++){
       	printf("%f \n",goal.trajectory.points[i].velocities[5]);
      }

      printf("---------------------------------------------------Acceleration--------------------------------------------------------------");
      for (int i=0; i<trajectory_size; i++){
       	printf("%f \n",goal.trajectory.points[i].accelerations[5]);
      }

    }else 
    {
      // We will have two waypoints in this goal trajectory
      goal.trajectory.points.resize(1);

      // Positions
      goal.trajectory.points[0].positions.resize(7);
      
      for (int i=0; i<7; i++)
      {
  			
        goal.trajectory.points[0].positions[i]=my_traj.trajectory[0].position_desired[joint_map[i]];
      }
      // Velocities
      goal.trajectory.points[0].velocities.resize(7);
        
      for (int i=0; i<7; i++)
      {
        goal.trajectory.points[0].velocities[i]=0;
      }
      // Accelerations
      goal.trajectory.points[0].accelerations.resize(7);
        
      for (int i=0; i<7; i++)
      {
        goal.trajectory.points[0].accelerations[i]=0;
      }
      goal.trajectory.points[0].time_from_start = ros::Duration(4);
    }
    

    
    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};


int main(int argc, char** argv)
{
      // Init the ROS node
      ros::init(argc, argv, "robot_driver");
     
        
        std::string file_name;
        //std::cout << "Making reader..." << std::endl;
        
        //std::cout << "...done" << std::endl;
        //std::cout << "Reading reference pose trajectory file..." << std::endl;
        
        

        //std::cout << "...done" << std::endl;
        //std::cout << "Printing trajectory to the screen..." << std::endl;
        //std::cout << my_traj << std::endl;
        //std::cout << "...done" << std::endl;
        
        std::cout << "Enter the name of trajecotry file you want to use: \n";
        getline (std::cin, file_name);
        std::cout << file_name << std::endl;
        XTF::Parser parser;
        XTF::Trajectory my_traj = parser.ParseTraj(file_name);
				//ROS_INFO("After parser");
        RobotArm arm;
        // Start the trajectory
        arm.startTrajectory(arm.armExtensionTrajectory(my_traj));
        // Wait for trajectory completion
        while(!arm.getState().isDone() && ros::ok() )
        {
          usleep(50000);
        }
      

 
}
