#include <iostream>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace std;

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    cout << "This should be moving left\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;


    char cmd[50];
		double c;
      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
      cmd_vel_pub_.publish(base_cmd);
      double start_time = clock();
			while(nh_.ok()){
      //std::cin.getline(cmd, 50);
      //base_cmd.angular.z = 0.5;
			c = abs(clock() - start_time);
			cout << abs(c) << endl;
			base_cmd.linear.y = -5 * c;
			cmd_vel_pub_.publish(base_cmd);
			//	base_cmd.linear.y = abs(sin(c));
			//	cout << i << endl;
			//cout << abs(sin(c)) << endl;
      //move forward
			//    if(cmd[0]=='+'){
			//      base_cmd.linear.x = 0.25;
			//   } 
					//turn left (yaw) and drive forward at the same time
		 	//   else if(cmd[0]=='l'){
			//     base_cmd.angular.z = 0.75;
			//     base_cmd.linear.x = 0.25;
			//   } 
			//turn right (yaw) and drive forward at the same time
			//   else if(cmd[0]=='r'){
			//     base_cmd.angular.z = -0.75;
			//     base_cmd.linear.x = 0.25;
			//   } 

      
      //publish the assembled command
    //  if(cmd[0]=='.'){
   //    break;
    //  }

		}

    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.driveKeyboard();

	return 0;
}

