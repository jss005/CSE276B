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
#define X_COMMON_AREA -8.5
#define Y_COMMON_AREA 1.14

#define SPIN_SPEED 0.6
#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 640

#define SWIVEL_TOLERANCE 0.1

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/*STATE VARIABLES*/
bool wait_mode;
bool explore_mode;
bool found_user;
bool requesting_interaction;
bool requesting_object;
bool requesting_object_name;

int explore_mode_enter_time;
bool obstacle_detected;

int swivel_dir;

/*FUNCTIONS*/

void initStates(){
  explore_mode_enter_time = 0;
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

  while (obstacle_detected){
    ROS_INFO("SWIVELING TO ENTER EXPLORE MODE");
    geometry_msgs::Twist cmd;
    cmd.angular.z = SPIN_SPEED;
    cmdpub_.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Hello, would anyone like to interact with me?");
  sound_play::SoundClient sc;
  sc.voiceSound("Hello, would anyone like to interact with me?").play();
  explore_mode_enter_time = ros::Time::now().toSec();
  explore_mode = true;
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

void requestInteraction(){
  ROS_INFO("Hello there, would you like to help me? If so, push on my base");
  sound_play::SoundClient sc;
  sc.voiceSound("Hello there, would you like to help me? If so, push on my base").play();
}

/*
Callback function for point clouds.  

cloud: the PointCloud message used to detect obstacles
*/
void cloud_cb (const PointCloud::ConstPtr& cloud) {
  int numValid = 0;
  float z_thresh = 2.0;

  //float x_thresh = 150;
  float x_thresh = 15000;
  int person_thresh = 30000;
  //int wall_thresh = 45000;
  int wall_thresh = 450000;

  //x and z position of the centroid
  float x = 0.0;
  float z = 0.0;

  //find potential obstacles
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
    ROS_INFO_THROTTLE(1, "Obstacle detected with %d points", numValid);
    obstacle_detected = true;

    //exit explore mode if a user is found
    if (explore_mode) {
      explore_mode = false;
      found_user = true;
      ROS_INFO_THROTTLE(1, "FOUND USER at x: %f z: %f, with %d points", x, z, numValid);
    }
    //if trying to interact with a person, give a direction to swivel to follow the person
    else if (requesting_interaction || requesting_object){
      //if within tolerance, do not swivel
      if (-SWIVEL_TOLERANCE < x && x < SWIVEL_TOLERANCE){
        swivel_dir = 0;
        ROS_INFO_THROTTLE(1, "NOT SWIVELING BECAUSE  x: %f ", x);
      }
      else {
        ROS_INFO_THROTTLE(1, "SWIVELING BECAUSE  x: %f ", x);
        swivel_dir = -x / std::abs(x);
      }
    }
  }
  //no obstacle detected, no swiveling
  else{ 
    ROS_INFO_THROTTLE(1, "No obstacle detected because %d points", numValid);
    obstacle_detected = false;
    swivel_dir = 0;
  }

  //TODO: test this to make sure it is valid
  //go back to explore mode if the person leaves
  //else if (numValid < person_thresh && requesting_interaction){
  //  requesting_interaction = false;
  //  enterExploreMode();
  //}

}

/*
Prompts the user to type in an object name, then looks it up online
and saves the result under learned_objects
*/
void lookupObject(){

  sound_play::SoundClient sc;
  sc.voiceSound("Please type in the name of this object on my keyboard").play();
  ROS_INFO("Please type in the name of this object on my keyboard");

  int result = -1;

  //loop until the user inputs a valid word - TODO: there should be exit criteria if user gives up
  while (result != 0){
    //prompt the user to input the object name
    std::string prompt = 
        std::string("python ~/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/init_object_prompt.py");
    system(prompt.c_str());

    //google the object
    std::string command = 
        std::string("python ~/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/googler.py ")
      + std::string("~/turtlebot_ws/src/turtlebot_apps/hernando/temp/obj_name.txt");
  
    result = system(command.c_str());
    ROS_INFO_THROTTLE(1, "Result from googler: %d", result);
    
    if (result != 0){
      sc.voiceSound("I could not find a definition for that. Can you give me a more basic description?").play();
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

  sound_play::SoundClient sc;

  while (1){

    ROS_INFO("commandLineCallBack: waiting for input");
    char input[100];
	  std::cin.getline(input, sizeof(input));

    bool success = false;

    //Navigate to the correct point of interest when input is given
    if (std::string(input).compare("explore") == 0 && wait_mode){
      success = moveToGoal(X_COMMON_AREA, Y_COMMON_AREA);
      if (success){
        wait_mode = false;
        enterExploreMode();
      }
    }
    else if (std::string(input).compare("return") == 0 && explore_mode){
      explore_mode = false;
      wait_mode = true;
      success = moveToGoal(X_LAB, Y_LAB);
      if (success){
        ROS_INFO("Press my bump sensor if you want to see what I learned");
        sc.voiceSound("Press my bump sensor if you want to see what I learned").play();
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

void bumperCallback(const kobuki_msgs::BumperEvent& bumperMessage){
  sound_play::SoundClient sc;
  // Check for pressed/not pressed
  if(bumperMessage.bumper == kobuki_msgs::BumperEvent::CENTER && 
     bumperMessage.state == kobuki_msgs::BumperEvent::PRESSED){
    ROS_INFO("BUMPER PRESSED!");
  
    //if requesting interaction, bump sensor indicates someone is willing to interact
    if (requesting_interaction){
      sc.voiceSound("Great! I'm trying to learn about different objects").play();
      ROS_INFO("Great! I'm trying to learn about different objects.");
      ROS_INFO("Please place anything on the ground in front of me so that I can see it");
      sc.voiceSound("Please place anything on the ground in front of me so that I can see it").play();
      ROS_INFO("Then push on my base to let me know I can start learning");
      sc.voiceSound("Then push on my base to let me know I can start learning").play();
      requesting_interaction = false;
      requesting_object = true;
    }
    //if requesting objects, bump sensor indicates an object was given
    else if (requesting_object) {
      ROS_INFO("Great, thank you!");
      sc.voiceSound("Great, thank you!").play();
      requesting_object = false;
      learnObject();
      requesting_object_name = true;
    }
    //if in wait mode, recite and show the objects that the robot has learned
    else if (wait_mode){
      system("python /home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/python_scripts/recite_objects.py");
    }
  }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "hernando");
	ros::NodeHandle n;
	ros::spinOnce();

  ros::Rate loop_rate(10);

  //subscribe to the /recognizer/output topic
  ros::Subscriber voiceSubscriber = n.subscribe("/recognizer/output", 100, voiceCallback);

  //subscribe to /PointCloud topic 
  ros::Subscriber PCSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, cloud_cb);

  //subscribe to the bumper topic
  ros::Subscriber BumperSubscriber = n.subscribe("/mobile_base/events/bumper", 100, bumperCallback);

  //create thread to listen to the command line
  std::thread threadObj(commandLineCallBack);

  initStates();

  //TODO: temporary, delete
  //explore_mode = true;
  //wait_mode = false;

  //main execution loop
  while (1){
   
    if (wait_mode){
      //don't do anything
    } 
    else if (found_user){
      found_user = false;
      requesting_interaction = true;
      requestInteraction();
    }
    else if (requesting_object){
      //wait for object and follow user
      spin_bot(swivel_dir, 0.1);
    }
    else if (requesting_interaction){
      //wait for interaction and follow user
      spin_bot(swivel_dir, 0.1);
    }
    //when requesting object name, prompt the user to input the object name
    else if (requesting_object_name){
      lookupObject();
      requesting_object_name = false;
      explore_mode = true;
    }
    else if (explore_mode){
      int now = ros::Time::now().toSec();
    
      if (now - explore_mode_enter_time > 30){
        enterExploreMode();
      }
    }
    
    //learnObject();

    ros::spinOnce();
    loop_rate.sleep();
    
  }

	return 0;
}

