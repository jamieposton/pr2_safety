#include <iostream>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;

//Both the example code and this were made in rosbuild, so that's a thing.
//Maybe change it to catkin at some point. That would probs be helpful.

//06.14/06.15 tried changing this package to catkin, and it wasn't working very well.
//Maybe something to do with the pr2 being made for groovy? Not really sure.
//The error it's giving has something to do with the app_manager
//Located in pr2_apps/pr2_app_manager. Not sure how to add that.

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class Torso{
private:
  TorsoClient *torso_client_;

public:
  //Action client initialization
  Torso(){
    
    torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);

    //wait for the action server to come up
    while(!torso_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the torso action server to come up");
    }
  }

  ~Torso(){
    delete torso_client_;
  }

  //tell the torso to go up
  void up(){

    pr2_controllers_msgs::SingleJointPositionGoal Up;
    Up.position = 0.190;  //all the way up is 0.2
    Up.min_duration = ros::Duration(2.0);
    Up.max_velocity = 1.0;
    
    ROS_INFO("Sending up goal");
    torso_client_->sendGoal(Up);
    //torso_client_->waitForResult();
  }

  //tell the torso to go down
  void down(){

    pr2_controllers_msgs::SingleJointPositionGoal Down;
    Down.position = 0.165;
    Down.min_duration = ros::Duration(2.0);
    Down.max_velocity = 1.0;

    ROS_INFO("Sending down goal");
    torso_client_->sendGoal(Down);
	 ROS_INFO("Sent down goal");
    //torso_client_->waitForResult();
  }    
};

class RobotDriver{
	private:
		//! The node handle we'll be using for the base
		ros::NodeHandle nh_;
		//! We will be publishing to the "/base_controller/command" topic
		ros::Publisher cmd_vel_pub_;

		//Action client for the joint trajectory action
		//used to trigger the arm movement action
		TrajClient* traj_client_;
		TrajClient* traj_client_L;

		PointHeadClient* point_head_client_;

	public:
		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			//set up the publisher for the cmd_vel topic
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

			//Initialize the client for the Action interface to the head controller
			point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

			// tell the action client that we want to spin a thread by default
			traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
			traj_client_L = new TrajClient("l_arm_controller/joint_trajectory_action", true);

			// wait for action server to come up
			while(!traj_client_->waitForServer(ros::Duration(5.0))){
			 ROS_INFO("Waiting for the joint_trajectory_action server");
			}
			while(!traj_client_L->waitForServer(ros::Duration(5.0))){
			 ROS_INFO("Waiting for the joint_trajectory_action server");
			}
			//wait for head controller action server to come up 
			while(!point_head_client_->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the point_head_action server to come up");
			}
		}

		~RobotDriver(){
			delete traj_client_;
			delete traj_client_L;
			delete point_head_client_;
		}

		//! Points the high-def camera frame at a point in a given frame  
		void lookAt(std::string frame_id, double x, double y, double z)
		{
			//the goal message we will be sending
			pr2_controllers_msgs::PointHeadGoal goal;

			//the target point, expressed in the requested frame
			geometry_msgs::PointStamped point;
			point.header.frame_id = frame_id;
			point.point.x = x; point.point.y = y; point.point.z = z;
			goal.target = point;

			//we are pointing the high-def camera frame 
			//(pointing_axis defaults to X-axis)
			goal.pointing_frame = "high_def_frame";

			//take at least 0.5 seconds to get there
			goal.min_duration = ros::Duration(0.75);

			//and go no faster than 1 rad/s
			goal.max_velocity = 0.4;

			//send the goal
			point_head_client_->sendGoal(goal);

			//wait for it to get there (abort after 2 secs to prevent getting stuck)
			point_head_client_->waitForResult(ros::Duration(1));
		}

		//! Shake the head from left to right n times  
		void shakeHead(int n)
		{
				//Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
				lookAt("base_link", 5.0, 10.0, 1.5);

				lookAt("base_link", 5.0, 10.0, -2.0);
		}

		void MoveArm(float* pos, float time){

			pr2_controllers_msgs::JointTrajectoryGoal goal;

			goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

			// First, the joint names, which apply to all waypoints
			goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
			goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
			goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
			goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
			goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
			goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
			goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

			// We will have three waypoints in this goal trajectory
			goal.trajectory.points.resize(1);

			// Positions
			goal.trajectory.points[0].positions.resize(7);
			for(int i = 0; i < 7; i++){
				goal.trajectory.points[0].positions[i] = pos[i]; 
			}

			// Velocities
			goal.trajectory.points[0].velocities.resize(7);
			for (size_t j = 0; j < 7; ++j){
				goal.trajectory.points[0].velocities[j] = 0.0;
			}
			// To be reached 1.5 seconds after starting along the trajectory
			goal.trajectory.points[0].time_from_start = ros::Duration(time);

			traj_client_->sendGoal(goal);

			//wait for it to get there (abort after 2 secs to prevent getting stuck)
			traj_client_->waitForResult(ros::Duration(time+1.0));
		}

		void MoveArmL(float* pos, float time){

			pr2_controllers_msgs::JointTrajectoryGoal goal;

			goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

			// First, the joint names, which apply to all waypoints
			goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
			goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
			goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
			goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
			goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
			goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
			goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

			// We will have three waypoints in this goal trajectory
			goal.trajectory.points.resize(1);

			// Positions
			goal.trajectory.points[0].positions.resize(7);
			for(int i = 0; i < 7; i++){
				goal.trajectory.points[0].positions[i] = pos[i]; 
			}

			// Velocities
			goal.trajectory.points[0].velocities.resize(7);
			for (size_t j = 0; j < 7; ++j){
				goal.trajectory.points[0].velocities[j] = 0.0;
			}
			goal.trajectory.points[0].time_from_start = ros::Duration(time);
			traj_client_L->sendGoal(goal);

			traj_client_L->waitForResult(ros::Duration(time+1.0));
		}

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState(){
    return traj_client_->getState();
  }

  actionlib::SimpleClientGoalState getStateL(){
    return traj_client_L->getState();
  }

  //! Loop forever while sending commands
  bool moveit(){
			//we will be sending commands of type "twist" for the base
			geometry_msgs::Twist base_cmd;

			cout << "Test Running. Hit Ctrl+C to End. " << endl;
			ros::Duration(2.0).sleep(); // sleep for 3 seconds

			//Initial values for base
			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
			cmd_vel_pub_.publish(base_cmd);

			while(nh_.ok()){
		
				float temp[7] = {0.0, 6.0, 1.5, -1.5, 0.5, 2.0, 0.0};
				MoveArmL(temp, 1.5);
				float tempa[7] = {0.0, -6.0, -3.2, -1.5, 0.0, 2.0, 0.0};
				MoveArm(tempa, 1.5);
				float tempb[7] = {0.0, -6.0, -3.2, 1.5, 0.0, 4.0, 0.0};
				MoveArm(tempb, 1.5);
				float tempc[7] = {0.0, 0.0, -3.2, 0.5, 0.0, 0.0, 0.0};
				MoveArm(tempc, 1.5);

				shakeHead(1);

				//Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
				lookAt("base_link", 5.0, 0.0, -2.0);

				base_cmd.linear.y = 1.0;
				for(int i = 0; i < 10; i++){	
					cmd_vel_pub_.publish(base_cmd);
					ros::Duration(0.1).sleep(); // sleep
				}
				//ros::Duration(1.0).sleep(); // sleep
				/*
				double nextTime = clock() + 5000;
				while(clock() < 5000){			
					double c = clock();
					//Divided by 10000 to try to exaggerate the slowing motion
					base_cmd.linear.y = -1*abs(sin(c/10000));
					cmd_vel_pub_.publish(base_cmd);
				}
				*/
				base_cmd.linear.y = 0;
				cmd_vel_pub_.publish(base_cmd);
			}
    return true;
  }
};

int main(int argc, char** argv){
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  Torso torso;
  
  torso.down();

  RobotDriver driver(nh);
  driver.moveit();

	return 0;
}

