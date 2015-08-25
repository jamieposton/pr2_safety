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
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;

//Both the example code and this were made in rosbuild, so that's a thing.
//Maybe change it to catkin at some point. That would probs be helpful.

//06.14/06.15 tried changing this package to catkin, and it wasn't working very well.
//Maybe something to do with the pr2 being made for groovy? Not really sure.
//The error it's giving has something to do with the app_manager
//Located in pr2_apps/pr2_app_manager. Not sure how to add that.

//Look into creating a new move arm function so there's no hesitation on the trajectories
//Publish like 4 or something points all at once to move in a circle
//instead of one, then hesistate, then another.

//Next step: add head movement in CircleArm function, or CirclePair function.
//Next step: look up how to move the grippers?

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
	private:
	GripperClient* gripper_client_;  

	public:
	//Action client initialization
	Gripper(){

		//Initialize the client for the Action interface to the gripper controller
		//and tell the action client that we want to spin a thread by default
		gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

		//wait for the gripper action server to come up 
		while(!gripper_client_->waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
		}
	}

	~Gripper(){
		delete gripper_client_;
	}

	//Open the gripper
	void open(){
		pr2_controllers_msgs::Pr2GripperCommandGoal open;
		open.command.position = 0.08;
		open.command.max_effort = -1.0;  // Do not limit effort (negative)

		ROS_INFO("Sending open goal");
		gripper_client_->sendGoal(open);
		gripper_client_->waitForResult();
		if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("The gripper opened!");
		else
			ROS_INFO("The gripper failed to open.");
	}

	//Close the gripper
	void close(float pos){
		pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
		squeeze.command.position = pos;
		squeeze.command.max_effort = 50.0;

		ROS_INFO("Sending squeeze goal");
		gripper_client_->sendGoal(squeeze);
		gripper_client_->waitForResult();
		if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("The gripper closed!");
		else
			ROS_INFO("The gripper failed to close.");
	}
};

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
  void down(){

    pr2_controllers_msgs::SingleJointPositionGoal Up;
    Up.position = 0.190;  //all the way up is 0.2
    Up.min_duration = ros::Duration(2.0);
    Up.max_velocity = 1.0;
    
    ROS_INFO("Sending up goal");
    torso_client_->sendGoal(Up);
    torso_client_->waitForResult();
  }

  //tell the torso to go down
  void up(){

    pr2_controllers_msgs::SingleJointPositionGoal Down;
    Down.position = 0.165;
    Down.min_duration = ros::Duration(2.0);
    Down.max_velocity = 1.0;

    ROS_INFO("Sending down goal");
    torso_client_->sendGoal(Down);
	 ROS_INFO("Sent down goal");
    torso_client_->waitForResult();
    cout << "Finished result" << endl;
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
		//TrajClient* traj_client_L;

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
			//traj_client_L = new TrajClient("l_arm_controller/joint_trajectory_action", true);

			// wait for action server to come up
			while(!traj_client_->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the joint_trajectory_action server");
			}
			/*
			while(!traj_client_L->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the joint_trajectory_action server");
			}
			*/
			
			//wait for head controller action server to come up 
			while(!point_head_client_->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the point_head_action server to come up");
			}
		}

		~RobotDriver(){
			delete traj_client_;
			//delete traj_client_L;
			delete point_head_client_;
		}

		//! Points the high-def camera frame at a point in a given frame  
		void lookAt(double *pos){

			//the goal message we will be sending
			pr2_controllers_msgs::PointHeadGoal goal;

			//the target point, expressed in the requested frame
			geometry_msgs::PointStamped point;
			point.header.frame_id = "base_link";
			point.point.x = pos[0]; point.point.y = pos[1]; point.point.z = pos[2];
			goal.target = point;

			//we are pointing the high-def camera frame 
			//(pointing_axis defaults to X-axis)
			goal.pointing_frame = "high_def_frame";

			//take at least 0.5 seconds to get there
			goal.min_duration = ros::Duration(0.75);

			//and go no faster than 1 rad/s
			goal.max_velocity = 0.75;

			//send the goal
			point_head_client_->sendGoal(goal);

			//wait for it to get there (abort after 2 secs to prevent getting stuck)
			point_head_client_->waitForResult(ros::Duration(1));
		}

		void MoveArm(float* pos, float time){

			pr2_controllers_msgs::JointTrajectoryGoal goal;

			goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);

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
/*
		void MoveArmL(float* pos, float time){

			pr2_controllers_msgs::JointTrajectoryGoal goal;

			goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);

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
		*/

		void CircleArm(float* center, float time){
			//Two points around the circle
			//Circle twice
			center[0] += 0.5;
			center[3] += 1.0;
			MoveArm(center, time);
			center[0] -= 0.5;
			center[3] -= 1.0;
			MoveArm(center, time);
			center[0] += 0.5;
			center[3] += 1.0;
			MoveArm(center, time);
			center[0] -= 0.5;
			center[3] -= 1.0;
			MoveArm(center, time);
			center[0] += 0.5;
			center[3] += 1.0;
			MoveArm(center, time);
		}

		void CirclePair(bool flip, float *top, float *bottom, double* lookTop, double* lookBot, float time, int choice){
			if(flip){
				if(choice == 1){
					lookAt(lookBot);
				}
				CircleArm(bottom, time);
				if(choice == 1){
					lookAt(lookTop);
				}
				CircleArm(top, time);
			}
			else{
				if(choice == 1){
					lookAt(lookTop);
				}
				CircleArm(top, time);
				if(choice == 1){
					lookAt(lookBot);
				}
				CircleArm(bottom, time);
			}

		}

		void CircleGroup(float pos[4][7], float time, bool* flip, double look[9][3], int& choice){
			CirclePair(flip[0], pos[3], pos[2], look[2], look[3], time, choice);
			CirclePair(flip[1], pos[1], pos[0], look[0], look[1], time, choice);
		}

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState(){
    return traj_client_->getState();
  }
/*
  actionlib::SimpleClientGoalState getStateL(){
    return traj_client_L->getState();
  }
  */
  

  //! Loop forever while sending commands
  bool moveit(int* choice){
			//we will be sending commands of type "twist" for the base
			geometry_msgs::Twist base_cmd;

			cout << "Test Running. Hit Ctrl+C to End. " << endl;
			//ros::Duration(2.0).sleep(); // sleep for 2 seconds

			//Initial values for base
			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
			cmd_vel_pub_.publish(base_cmd);

/*
			if(choice[1] == 1){
				float temp[7] = {0.0, 6.0, 1.5, -1.5, 0.5, 2.0, 0.0};
				MoveArmL(temp, 1.5);
			}
			*/

			int n = 0;
			bool flip[2] = {false, false};

			while(nh_.ok()){

				float pos[4][7] = {
					{-0.5, 0.0, -1.6, -2.0, 0.0, 1.0, 1.5},
					{-0.3, 0.0, -1.6, -1.0, 0.0, 1.0, 1.5},
					{-1.0, 0.0, -1.6, -2.0, 0.0, 1.0, 1.5},
					{-0.8, 0.0, -1.6, -1.0, 0.0, 1.0, 1.5}
				};

				double look[9][3] = {
					{5.0, 2.0, -2.0},
					{5.0, 1.0, -2.0},
					{5.0, -4.0, -2.0},
					{4.0, -3.0, -2.0},

					{5.0, 10.0, -2.0},
					{5.0, -10.0, -2.0},
					{5.0, 10.0, 1.2},
					{5.0, 10.0, -2.0},
					{5.0, 0.0, -2.0}
				};

				if(choice [1] == 1){

					CircleGroup(pos, 1.0, flip, look, choice[2]);
				}

				if(n%3 == 2 && choice[2] == 1){
					lookAt(look[4]);
				}

				if(n%3 == 2 && choice[2] == 1){
					lookAt(look[5]);
				}

				if(choice[2] == 1){
					lookAt(look[6]);
					lookAt(look[7]);
					lookAt(look[8]);
				}

				if(choice[0] == 1){
					base_cmd.linear.y = 0.5;
					for(int i = 0; i < 20; i++){	
						cmd_vel_pub_.publish(base_cmd);
						ros::Duration(0.1).sleep(); // sleep
					}
				}
				ros::Duration(1.0).sleep(); // sleep
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
				if(choice[1] == 0){
					ros::Duration(3.0).sleep();
				}

				n++;
				if(flip[0] == false){
					if(flip[1] == false){
						flip[1] = true;
					}
					else{
						flip[0] = true;
						flip[1] = false;
 					}
				}
				else{
					if(flip[1] == false){
						flip[1] = true;
					}
					else{
						flip[0] = false;
						flip[1] = false;
 					}
				}
			}
    return true;
  }
};

void printHelp(){
	cout << "Possible options" << endl;
	cout << "b to run the base" << endl;
	cout << "a to run the arms" << endl;
	cout << "h to run the head" << endl;
	cout << "default is to sweep vertically" << endl;
}

int main(int argc, char** argv){

	int choice[3] = {0,0,0};
	//First is base
	//Second is arm
	//Third is head

	//Fourth used to be vertical or horizontal arm sweeps

	if(argc <= 1){
		cout << "Error: not enough arguments" << endl;
		printHelp();
		return 0;
	}

	for(int i = 1; i < argc; i++){
		switch(argv[i][0]){
			case 'b':
				choice[0] = 1;
				break;
			case 'a':
				choice[1] = 1;
				break;
			case 'h':
				choice[2] = 1;
				break;
			default:
				cout << "Incorrect argument given. Try again." << endl;
				printHelp();
				return 0;
		}
	}

	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;
/*
	cout << "Before Torso" << endl;
	Torso torso;
	torso.up();
	cout << "After Torso" << endl;
	*/
/*
	Gripper gripper;
	gripper.open();

	cout << "After Gripper open" << endl;

	cout << "Adjusting gripper position. o to open, c to close, e to exit and continue program." << endl;
	bool done = true;
	float x = 0.08;
	char input;
	while(done){
		ros::Duration(0.5).sleep();
		cout << "Enter Value: "; 
		cin >> input;
		switch(input){
			case 'o':
				x += 0.01;
				gripper.close(x);
				break;
			case 'c':
				x += -0.01;
				gripper.close(x);
				break;
			case 'e':
				done = false;
				break;
			default:
				cout << "Command not recognized. Try again." << endl;
				break;
		}
	}
*/
	RobotDriver driver(nh);
	driver.moveit(choice);

	return 0;
}