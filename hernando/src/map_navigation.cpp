/*
File: hernando.cpp
Author: Jeremy Smith
Date: 3/22/2018

This program defines SpockBot, an exploratory robot that asks people for objects to learn and
reports back about them.

Usage:
roscore
roslaunch turtlebot_bringup minimal.launch
roslaunch astra_launch astra_pro_amcl.launch
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/maps/third.yaml
roslaunch turtlebot_rviz_launchers view_navigation.launch
<localize robot to starting location then exit> 
rosrun image_view image_view image:=/camera/rgb/image_raw theora

<navigation to hernando/temp>
rosrun image_view image_saver image:=/camera/rgb/image_raw _save_all_image:=false _filename_format:=frame%04i.jpg __name:=image_saver

rosrun hernando hernando
 */

/*INCLUDES*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sound_play/sound_play.h"

#include <std_msgs/String.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <nodelet/nodelet.h>
#include <kobuki_msgs/BumperEvent.h>


#include <thread>


/*DEFINES*/
//coordinates for points of interests
#define X_LAB -1.9
#define Y_LAB -0.04
//#define X_COMMON_AREA -8.5
//#define Y_COMMON_AREA 1.14
#define X_COMMON_AREA -31.6
#define Y_COMMON_AREA -1.1

#define SPIN_SPEED 0.6
#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 640

#define SWIVEL_TOLERANCE 0.05

#define EXPLORE_MODE_REPROMPT 30
#define INTERACTION_TIMEOUT 30

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/*STATE VARIABLES*/
bool wait_mode;
bool explore_mode;
bool found_user;
bool requesting_interaction;
bool requesting_object;
bool requesting_object_name;
bool offering_information;

int explore_mode_enter_time;
int interaction_enter_time;
bool obstacle_detected;

int swivel_dir;

/*GLOBALS*/

/*FUNCTIONS*/

void initStates(){
  explore_mode_enter_time = 0;
  interaction_enter_time = 0;
  wait_mode = true;
  explore_mode = false;

  obstacle_detected = true;
  swivel_dir = 0;
}

/*
Resets the timer for entering explore mode, enters explore mode,
and turns the robot until there are no longer any objects in view
*/
void enterExploreMode(){
  ros::Rate loop_rate(10);
  ros::NodeHandle n;
  ros::Publisher cmdpub_ = n.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1000);

  //assume by default there are obstacles in view until cloud_cb says otherwise
  obstacle_detected = true;

  while (obstacle_detected){
    geometry_msgs::Twist cmd;
    cmd.angular.z = SPIN_SPEED;
    cmdpub_.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }

  //ensure all other states are false
  wait_mode = false;
  found_user = false;
  requesting_interaction = false;
  requesting_object = false;
  requesting_object_name = false;
  offering_information = false;
  

  ROS_INFO("Greetings. I require someone to assist me.");
  system("espeak -v m4 -s 120 'Greetings. I require someone to assist me.'");
  explore_mode_enter_time = ros::Time::now().toSec();
  explore_mode = true;
}

/*
Resets the time for beginning an interaction
*/
void resetInteractionTime(){
  interaction_enter_time = ros::Time::now().toSec();
}

/*
Callback for voices
*/
void voiceCallback(const std_msgs::String& msg){

  std::string data = msg.data;

  //debug
  const char* temp = msg.data.c_str();
  ROS_INFO_THROTTLE(1, "Heard the following:\n%s", temp);

}

/*
Makes the robot request an interaction with a human that has been noticed
*/
void requestInteraction(){
  resetInteractionTime();
  requesting_interaction = true;

  ROS_INFO("Greetings, I require your assistance. To proceed, push on my base.");
  system("espeak -v m4 -s 120 'Greetings, I require your assistance. To proceed, push on my base.'");
}

/*
Makes the robot request an object from a human that has agreed to interact
*/
void requestObject(){
  resetInteractionTime();
  requesting_object = true;

  system("espeak -v m4 -s 120 'I am attempting to scan different objects'");
  ROS_INFO("I am attempting to scan different objects.");
  ROS_INFO("Please place any object on the ground in front of me so that I can see it on my screen"); 
  system("espeak -v m4 -s 120 'Please place any object on the ground in front of me so that I can see it on my screen'"); 
  ROS_INFO("Then push on my base to initiate the scan");
  system("espeak -v m4 -s 120 'Then push on my base to initiate the scan'");
}

/*
Callback function for point clouds.  

Sets the swivel direction that the robot should move in if a person is detected.

cloud: the PointCloud message used to detect obstacles
*/
void cloud_cb (const PointCloud::ConstPtr& cloud) {
  int numValid = 0;
  float z_thresh = 2.0;

  float x_thresh = 250;
  //float x_thresh = 15000;
  int person_thresh = 30000;
  //int wall_thresh = 45000;
  int wall_thresh = 450000;

  //x and z position of the centroid
  float x = 0.0;
  float z = 0.0;

  //find potential people
  for (int k = 0; k < IMAGE_HEIGHT; k++){
    for (int i = 0; i < IMAGE_WIDTH; i++){
      const pcl::PointXYZ &pt = cloud->points[640*(180+k)+(i)];

      //ignore invalid points and points out of range
      if (isnan(pt.z) || pt.z > z_thresh)
        continue;

      if (i < (IMAGE_WIDTH/2) - x_thresh || i > (IMAGE_WIDTH/2) + x_thresh)
        continue;

      x += pt.x;
      z += pt.z;

      numValid++;
    }
  }

  //average out x and z points within range
  x /= numValid;
  z /= numValid;

  if (numValid > person_thresh && numValid < wall_thresh){
    //ROS_INFO_THROTTLE(1, "Obstacle detected with %d points", numValid);
    obstacle_detected = true;

    //exit explore mode if a user is found
    if (explore_mode) {
      explore_mode = false;
      found_user = true;
      //ROS_INFO_THROTTLE(1, "FOUND USER at x: %f z: %f, with %d points", x, z, numValid);
    }
    //if trying to interact with a person, give a direction to swivel to follow the person
    else if (requesting_interaction || requesting_object){
      //if within tolerance, do not swivel
      if (-SWIVEL_TOLERANCE < x && x < SWIVEL_TOLERANCE){
        swivel_dir = 0;
        //ROS_INFO_THROTTLE(1, "NOT SWIVELING BECAUSE  x: %f ", x);
      }
      else {
        //ROS_INFO_THROTTLE(1, "SWIVELING BECAUSE  x: %f ", x);
        swivel_dir = -x / std::abs(x);
      }
    }
  }
  //no obstacle detected, no swiveling
  else{ 
    //ROS_INFO_THROTTLE(1, "No obstacle detected because %d points", numValid);
    obstacle_detected = false;
    swivel_dir = 0;
  }
}

/*
Prompts the user to type in an object name, then looks it up online
and saves the result under learned_objects
*/
void lookupObject(){

  system("espeak -v m4 -s 120 'Please type in the name of this object on my keyboard'");
  ROS_INFO("Please type in the name of this object on my keyboard");

  int result = -1;

  //loop until the user inputs a valid word or if external termination of requesting_object_name
  while (result != 0 && requesting_object_name){
    //prompt the user to input the object name
    std::string prompt = 
        std::string("python ~/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/init_object_prompt.py");
    system(prompt.c_str());

    //check if the interaction timed out
    int now = ros::Time::now().toSec();
    if (now - interaction_enter_time > INTERACTION_TIMEOUT){
      break;
    }

    //google the object
    std::string command = 
        std::string("python ~/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/googler.py ")
      + std::string("~/turtlebot_ws/src/turtlebot_apps/hernando/temp/obj_name.txt");
  

    result = system(command.c_str());
    ROS_INFO_THROTTLE(1, "Result from googler: %d", result);
    
    if (result != 0){
      system("espeak -v m4 -s 120 'I could not find a definition for that. I require a more basic description?'");
    }
    else {
      //CODE FOR NORMAL OPERATION -- THANK AND OFFER
      system("espeak -v m4 -s 120 'Thank you. You may press on my base to hear the results of the scan.'");
      offering_information = true;
      resetInteractionTime();
      
      
    
      //CODE FOR EXPERIMENT 1 -- THANK AND SHARE
      /*
      system("espeak -v m4 -s 120 'Scans completed.'");
      system("python /home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/recite_objects.py single");
      system("espeak -v m4 -s 120 'Do you have additional objects? Press my base once you have placed something in front of me'");
      system("espeak -v m4 -s 120 'If you have no other objects, liv long and prosper.'");
      requesting_object_name = false;
      requesting_object = true;
      resetInteractionTime();
      */
      
      
      //CODE FOR EXPERIMENT 2 -- THANK AND NO SHARE
      /*
      system("espeak -v m4 -s 120 'Thank you'");
      system("espeak -v m4 -s 120 'Do you have additional objects? Press my base once you have put something in front of me'");
      system("espeak -v m4 -s 120 'If you have no other objects, liv long and prosper.'");
      requesting_object_name = false;
      requesting_object = true;
      resetInteractionTime();
      */
    }
  }


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
    cmd.angular.z = direction * SPIN_SPEED;
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

  resetInteractionTime();

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
  resetInteractionTime();
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
  
  resetInteractionTime();
  
}

/*
Attempts to move the robot to a location on the saved map specified by
xGoal and yGoal

Code is from gaitech_edu tutorial
http://edu.gaitech.hk/turtlebot/map-navigation.html
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

  //sound_play::SoundClient sc; 

  while (1){

    ROS_INFO("commandLineCallBack: waiting for input");
    char input[100];
	  std::cin.getline(input, sizeof(input));

    bool success = false;

    //Navigate to the correct point of interest when input is given
    if (std::string(input).compare("explore") == 0 && wait_mode){
      system("espeak -v m4 -s 120 'One to beam down'");
      success = moveToGoal(X_COMMON_AREA, Y_COMMON_AREA);
      if (success){
        wait_mode = false;
        enterExploreMode();
      }
    }
    else if (std::string(input).compare("return") == 0 && explore_mode){
      explore_mode = false;
      wait_mode = true;
      system("espeak -v m4 -s 120 'One to beam up'");
      success = moveToGoal(X_LAB, Y_LAB);
      if (success){
        ROS_INFO("Press my bump sensor to listen to my report.");
        //sc.voiceSound("Press my bump sensor if you want to see what I learned").play();
        system("espeak -v m4 -s 120 'Press my bump sensor to listen to my report'");
      }
      else{
        wait_mode = false;
        enterExploreMode();
      }
    }
    else{
      ROS_INFO("I can't perform that action right now");
    }

  }

}

/*
Callback for the bump sensor.
Messages indicate that a user is interacting with the robot and an appropriate action
should be taken
*/
void bumperCallback(const kobuki_msgs::BumperEvent& bumperMessage){
  sound_play::SoundClient sc; 

  // Check for pressed/not pressed
  if(bumperMessage.bumper == kobuki_msgs::BumperEvent::CENTER && 
     bumperMessage.state == kobuki_msgs::BumperEvent::PRESSED){
    ROS_INFO("BUMPER PRESSED!");
  
    //if requesting interaction, bump sensor indicates someone is willing to interact
    if (requesting_interaction){
      requesting_interaction = false;
      requestObject();
    }
    //if requesting objects, bump sensor indicates an object was given
    else if (requesting_object) {
      ROS_INFO("Initiating scans");
      system("espeak -v m4 -s 120 'Initiating scans'");
      requesting_object = false;
      learnObject();
      requesting_object_name = true;
    }
    //if in wait mode, recite and show the objects that the robot has learned
    else if (wait_mode){
      system("python /home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/recite_objects.py");
    }
    //if offering information, recite the definition of the last object learned
    else if (offering_information){
      system("python /home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/recite_objects.py single");
    }
  }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "hernando");
	ros::NodeHandle n;
	ros::spinOnce();

  ros::Rate loop_rate(10);
  ros::Publisher cmdpub_ = n.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1000);

  //subscribe to the /recognizer/output topic
  ros::Subscriber voiceSubscriber = n.subscribe("/recognizer/output", 100, voiceCallback);

  //subscribe to /PointCloud topic 
  ros::Subscriber PCSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, cloud_cb);

  //subscribe to the bumper topic
  ros::Subscriber BumperSubscriber = n.subscribe("/mobile_base/events/bumper", 100, bumperCallback);

  //create thread to listen to the command line
  std::thread threadObj(commandLineCallBack);

  initStates();

  //uncomment to begin in explore mode
  //enterExploreMode();
  //wait_mode = false;

  //main execution loop
  while (1){
   
    //return to explore mode if an interaction times out
    int now = ros::Time::now().toSec();
    if ((requesting_object || requesting_interaction || requesting_object_name || offering_information) &&
        now - interaction_enter_time > INTERACTION_TIMEOUT){
      initStates();
      enterExploreMode();
    }

    //take appropriate behavior based on states    

    if (wait_mode){
      //don't do anything
    } 
    else if (found_user){
      found_user = false;
      requestInteraction();
    }
    else if (requesting_object){
      //wait for object and follow user
      geometry_msgs::Twist cmd;
      cmd.angular.z = swivel_dir * SPIN_SPEED;
      cmdpub_.publish(cmd);
    }
    else if (requesting_interaction){
      //wait for interaction and follow user
      geometry_msgs::Twist cmd;
      cmd.angular.z = swivel_dir * SPIN_SPEED;
      cmdpub_.publish(cmd);
    }
    //when requesting object name, prompt the user to input the object name
    else if (requesting_object_name){
      lookupObject();
      requesting_object_name = false;
      //enterExploreMode();
    }
    else if (offering_information){
      //wait for prompt and follow user
      geometry_msgs::Twist cmd;
      cmd.angular.z = swivel_dir * SPIN_SPEED;
      cmdpub_.publish(cmd);
    
    }
    //revocalize the explore mode message if robot has been in explore mode for too long
    else if (explore_mode){
      int now = ros::Time::now().toSec();
      if (now - explore_mode_enter_time > EXPLORE_MODE_REPROMPT){
        enterExploreMode();
        system("espeak -v m4 -s 120 'Logic dictates that helping me will make me go away.'");
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
    
  }

	return 0;
}

