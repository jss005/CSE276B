/*
 *TODO: header
 */

/*INCLUDES*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sound_play/sound_play.h"

#include <std_msgs/String.h>
#include <string>

#include <thread>


/*DEFINES*/
//coordinates for points of interests
#define X_LAB 6.65
#define Y_LAB -1.00
#define X_COMMON_AREA 9.47
#define Y_COMMON_AREA 0.68

#define SPIN_SPEED 0.5

/*STATE VARIABLES*/
bool wait_mode;
bool explore_mode;
bool requesting_object_name;

/*FUNCTIONS*/

void initStates(){
  wait_mode = true;
  explore_mode = false;
}

/*
Callback for voices
*/
void voiceCallBack(const std_msgs::String& msg){

  std::string data = msg.data;

  //debug
  const char* temp = msg.data.c_str();
  ROS_INFO_THROTTLE(1, "Heard the following:\n%s", temp);

}

/*
Prompts the user to type in an object name, then looks it up online
and saves the result under learned_objects
*/
void lookupObject(){

  sound_play::SoundClient sc;
  sc.voiceSound("Please type in the name of this object on my keyboard").play();

  //prompt the user to input the object name
  std::string prompt = 
      std::string("python ~/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/init_object_prompt.py");
  system(prompt.c_str());

  //google the object
  std::string command = 
      std::string("python ~/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/googler.py ")
    + std::string("~/turtlebot_ws/src/turtlebot_apps/hernando/temp/obj_name.txt");
  system(command.c_str());

  //TODO: add code to the python to move all pictures in the temp directory
  //into the learned_objects/<obj_name> directory. Note that this expects image_saver to have been run
  //from the hernando/temp directory
  //rosrun image_view image_saver image:=/camera/rgb/image_raw _save_all_image:=false 
  //_filename_format:=frame%04i.jpg __name:=image_saver
}

/*
Spin the robot in the given direction for a given amount of seconds
*/
void spin_bot(int direction, double duration_sec){
  ros::Rate loop_rate(10);
  ros::NodeHandle n;
  ros::Publisher cmdpub_ = n.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1000);


  double t0 = ros::Time::now().toSec();
  while (ros::Time::now().toSec() - t0 < duration_sec){
    geometry_msgs::Twist cmd;
    cmd.angular.z = direction * 0.3 * 2;
    cmdpub_.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/*
Moves the robot forwards or backwards for a given amount of seconds
*/
void move_bot(int direction, double duration_sec){
  ros::Rate loop_rate(10);
  ros::NodeHandle n;
  ros::Publisher cmdpub_ = n.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1000);


  double t0 = ros::Time::now().toSec();
  while (ros::Time::now().toSec() - t0 < duration_sec){
    geometry_msgs::Twist cmd;
    cmd.linear.x = direction * 0.1;
    cmdpub_.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/*
take images of the object. If the object is placed on the ground then move the turtlebot between
pictures. If I put on a tray, then just take one
*/
void learnObject(){
  ros::Rate wait(0.5);

  system("rosservice call /image_saver/save");
  spin_bot(1, 0.5);
  wait.sleep();
  system("rosservice call /image_saver/save");
  spin_bot(-1, 0.5);
  spin_bot(-1, 0.5);
  wait.sleep();
  system("rosservice call /image_saver/save");
  spin_bot(1, 0.5);
  wait.sleep();
  system("rosservice call /image_saver/save");

  move_bot(-1, 2.0);
  wait.sleep();
  system("rosservice call /image_saver/save");
  spin_bot(-1, 0.5);
  wait.sleep();
  system("rosservice call /image_saver/save");
  spin_bot(1, 0.5);
  spin_bot(1, 0.5);
  wait.sleep();
  system("rosservice call /image_saver/save");
  spin_bot(-1, 0.5);
  wait.sleep();
  system("rosservice call /image_saver/save");
  move_bot(1, 2.0);
  wait.sleep();
  system("rosservice call /image_saver/save");
  
  

  system("rosservice call /image_saver/save");
}

/*
Attempts to move the robot to a location on the saved map specified by
xGoal and yGoal

Code is from gaitech_edu tutorial
*/
bool moveToGoal(double xGoal, double yGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}

/*
Listens to the command line for commands to go exploring and to return

Explore commands can only be given while the robot is waiting.
Return commands can only be given while the robot is exploring.
*/
void commandLineCallBack(){

  while (1){

    ROS_INFO("commandLineCallBack: waiting for input");
    char input[100];
	  std::cin.getline(input, sizeof(input));

    //Navigate to the correct point of interest when input is given
    if (std::string(input).compare("explore") == 0 && wait_mode){
      wait_mode = false;
      explore_mode = true;
      moveToGoal(X_COMMON_AREA, Y_COMMON_AREA);
    }
    else if (std::string(input).compare("return") == 0 && explore_mode){
      explore_mode = false;
      wait_mode = true;
      moveToGoal(X_LAB, Y_LAB);
    }
    else{
      ROS_INFO("I can't perform that action right now");
    }

  }

}


int main(int argc, char** argv){
	ros::init(argc, argv, "hernando");
	ros::NodeHandle n;
	ros::spinOnce();

  ros::Rate loop_rate(10);

  //subscribe to the /recognizer/output topic
  ros::Subscriber voiceSubscriber = n.subscribe("/recognizer/output", 100, voiceCallBack);

  //create thread to listen to the command line
  std::thread threadObj(commandLineCallBack);

  initStates();

  //main execution loop
  while (1){
    
    if (requesting_object_name){
      lookupObject();
      requesting_object_name = false;
      explore_mode = true;
    }
    //ROS_INFO("Spinning");
    
    learnObject();
    break;

    ros::spinOnce();
    loop_rate.sleep();
    
  }

	return 0;
}

