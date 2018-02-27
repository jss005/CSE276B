/*
Name: trash_bot.cpp
Author: Jeremy Smith
Date: 2/26/2018

Description: This program has the turtlebot roam around in search of trash on the ground (represented
as a red object). When the trash is found, the robot will approach it and ask for help picking it up.
Upon receiving help, the robot will move side to side "happily" then ask if it is recyclable or trash.
When told which one it is, the robot will seek out the appropriate trash can and approach it. Then, the
robot will ask for help throwing the trash away. Finally, the robot will thank the person for helping.

Usage:
roscore
roslaunch turtlebot_bringup minimal.launch
roslaunch astra_launch astra_pro.launch

rosrun cmvision colorgui image:=/camera/rgb/image_raw
<close it when done>
roslaunch cmvision cmvisionlaunch image:=/camera/rgb/image_raw
<Ctrl-C>
rosparam set /cmvision/color_file ~/turtlebot_ws/src/cmvision/colors.txt
rosrun cmvision cmvision image:=/camera/rgb/image_raw

roslaunch trash_bot recognizer.launch

rosrun sound_play soundplay_node.py

rosrun trash_bot trash_bot
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <nodelet/nodelet.h>

#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>

#include <std_msgs/String.h>
#include <string>

#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 640
#define SPIN_SPEED 0.5

// STATE VARIABLES
bool spin_mode;
bool found_trash_mode;
bool pickup_mode;
bool given_trash_mode;
bool asking_destination_mode;
bool at_bin_mode;

////////

// Colors to follow
int COLOR_BLUE_R  = 0;
int COLOR_BLUE_G  = 0;
int COLOR_BLUE_B  = 255;

int COLOR_BLACK_R  = 0;
int COLOR_BLACK_G  = 0;
int COLOR_BLACK_B  = 0;

int follow_color_r;
int follow_color_g;
int follow_color_b;

//misc
int avoid_direction; //which way to turn to avoid an obstacle. + for left, - for right
int spin_direction; //direction to spin when searching. Default 1, but set to be opposite of avoid_direction
int spin_mode_enter_time;

int done_criteria = 80000; //Area of color that must be present to decide we are at the goal
double center = 320.0; //Center of the screen
double goal_x = 0.0; //Average x position of color blobs

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher cmdpub_; //publisher for movement commands

/*
Puts the robot into spin mode. Logs the time entered so that wandering can be implemented
*/
void enterSpinMode(){
  if (!spin_mode){
    spin_mode_enter_time = ros::Time::now().toSec();
  }
  spin_mode = true;
}


/*
Function to initialize states and other variables
*/
void blobFollowerInit() {
  enterSpinMode();
  
  found_trash_mode = false;
  pickup_mode = false;
  given_trash_mode = false;
  asking_destination_mode = false;
  at_bin_mode = false;

  spin_direction = 1;
}

/*
Spin the robot in the given direction for a given amount of seconds
*/
void spin_bot(int direction, int duration_sec){
  ros::Rate loop_rate(10);
  ros::NodeHandle n;
  ros::Publisher cmdpub_ = n.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1000);
  

  int t0 = ros::Time::now().toSec();
  while (ros::Time::now().toSec() - t0 < duration_sec){
    geometry_msgs::Twist cmd;
    cmd.angular.z = direction * SPIN_SPEED * 3;
    cmdpub_.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
}


/*
Callback function for color blobs. Searches for the target's color and sets spin_mode to false
if it finds it, causing the robot to pursue the target. Otherwise, tells the robot to spin.
If the goal is within range of the robot, measured by the area of color in the image, the
robot is told to finish.

blobsIn: the color blob message
*/
void blobsCallBack (const cmvision::Blobs& blobsIn) {
    goal_x = 0.0;
    int color_blob_count = 0;
    int color_area = 0;
    //int color_area_thresh = 2000; //can be used in noisy environments where there may be small patches of color
    int color_area_thresh = 0; //for clean environments

    //DO NOT go goal searching if found trash and waiting for pickup
    if (pickup_mode || found_trash_mode || asking_destination_mode || given_trash_mode || at_bin_mode){
      return;
    }

    //find all the blobs that are COLOR and sum their x positions
    for (int i = 0; i < blobsIn.blob_count; i++){
      if (blobsIn.blobs[i].red == 255 && blobsIn.blobs[i].green == 0 && blobsIn.blobs[i].blue == 0){
        goal_x += blobsIn.blobs[i].x;
        color_blob_count++;
        color_area += blobsIn.blobs[i].area;

        ROS_INFO_THROTTLE(1, "Bottom of blob is: %d", blobsIn.blobs[i].bottom);

        //stop when we are near the trash (color blob is at bottom of screen)
        if (blobsIn.blobs[i].bottom >= 479 && blobsIn.blobs[i].top >= 440){
          found_trash_mode = true;
          return;
        }
      }
    }


    //get the average x position of all COLOR blobs
    if (color_blob_count > 0 && color_area > color_area_thresh){
      //End spin_mode since the goal has been detected
      spin_mode = false;
      goal_x = (goal_x / color_blob_count);
    }
    else{ //if no COLOR blobs were found, go back to spin mode
      goal_x = 0.0;
      enterSpinMode();
    }
}

/*
Callback function for point clouds.  Checks to see if there are any obstacles in range.
If so, and the robot is not just spinning, sets avoid_mode and indicates which direction
to move in.  Also sets the direction to start looking for the target once the obstacle has
been avoided.

cloud: the PointCloud message used to detect obstacles
*/
void cloud_cb (const PointCloud::ConstPtr& cloud) {
  int numValid = 0;
  float z_thresh = 0.7;

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

      x += pt.x;
      z += pt.z;

      numValid++;
    }
  }

  //average out x and z points within range
  x /= numValid;
  z /= numValid;

  ROS_INFO_THROTTLE(1, "Centroid detected at x: %f z: %f, with %d points", x, z, numValid);

  //exit pickup_mode when the person gives the robot the trash object
  if (pickup_mode && numValid > 11000 && z > 0.4 && z < 0.7){
    pickup_mode = false;
    given_trash_mode = true;
    return;
  }

  //reset states when the trash is thrown away
  if (at_bin_mode && numValid < 1000 && (z < 0.4 || z > 0.7 || isnan(z)) ){
    ROS_INFO_THROTTLE(1, "Exiting because numValid = %d, z = %f", numValid, z);
    ROS_INFO("I appreciate all the help");
    system("rosrun sound_play say.py 'I appreciate all the help'");
    blobFollowerInit();
    return;
  }

}

/*
Callback for heard voices. Recognizes words in the following patterns:
trash
garbage
recycle
recyclable
that is trash
that is recyclable
*/
void voiceCallBack(const std_msgs::String& msg){

  //exit callback if we don't care what people say
  if (!asking_destination_mode){
    return;
  }

  std::string data = msg.data;

  //debug
  const char* temp = msg.data.c_str();
  ROS_INFO_THROTTLE(1, "Heard the following:\n%s", temp);

  //listen for the human telling the robot if they should throw it in the
  //trash or recycle bin.
  if (data.find("recycle") != std::string::npos ||
      data.find("recyclable") != std::string::npos) {

    asking_destination_mode = false;

    ROS_INFO("Alright I will throw this in the recycle bin");
    system("rosrun sound_play say.py 'Alright I will throw this in the recycle bin'");
    system("rosrun blob_follower blob_follower BLUE");
    ROS_INFO("Please help me throw this into the recycle bin");
    system("rosrun sound_play say.py 'Please help me throw this into the recycle bin'");
    at_bin_mode = true;
    spin_bot(1, 4); //"offer" the object
  }
  else if (data.find("trash") != std::string::npos ||
           data.find("garbage") != std::string::npos){
  
    asking_destination_mode = false;

    ROS_INFO("Alright I will throw this in the trash bin");
    system("rosrun sound_play say.py 'Alright I will throw this in the trash bin'");
    system("rosrun blob_follower blob_follower BLACK");
    ROS_INFO("Please help me throw this into the trash bin");
    system("rosrun sound_play say.py 'Please help me throw this into the trash bin'");
    at_bin_mode = true;
    spin_bot(1, 4); //"offer" the object
  }
  

}


/*
Implements some behavior for when a trash object is found
*/
void foundTrash(){
  ROS_INFO("I found some trash");
  system("rosrun sound_play say.py 'I found some trash'");
  ROS_INFO("Will you help me pick it up?");
  system("rosrun sound_play say.py 'Will you help me pick it up'");
  ROS_INFO("Done speaking");

  found_trash_mode = false;
  pickup_mode = true;

}


int main (int argc, char** argv) {
  ros::init(argc, argv, "trash_bot");

  // Create handle that will be used for both subscribing and publishing. 
  ros::NodeHandle n;
  
  //ROS_INFO("Exited from other program");

  //subscribe to /PointCloud topic 
  ros::Subscriber PCSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, cloud_cb);

  //subscribe to /blobs topic 
  ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);

  //subscribe to the /recognizer/output topic
  ros::Subscriber voiceSubscriber = n.subscribe("/recognizer/output", 100, voiceCallBack);

  // publishing the geometry message twist message
  ros::Publisher cmdpub_ = n.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1000);

  ROS_INFO("Program started!");

  //initiate the robot state
  blobFollowerInit();
  
  // Set the loop frequency in Hz.
  ros::Rate loop_rate(10);
   
  //runtime loop
  while(ros::ok()){
    geometry_msgs::Twist cmd;

        
    //found trash, ask for help picking it up
    if (found_trash_mode){
      foundTrash();
    }
    //wait until person helps out
    else if (pickup_mode){
      //wait for the person to help pick up the trash
    }
    //received trash. Spin happily
    else if (given_trash_mode){
      ROS_INFO("Thank you");
      system("rosrun sound_play say.py 'Thank you'");
      spin_bot(-1, 1);
      spin_bot(1, 1);
      spin_bot(-1, 1);
      spin_bot(1, 1);
      given_trash_mode = false;
      asking_destination_mode = true;

      //ask once if this is trash or recyclable
      system("rosrun sound_play say.py 'Is this recyclable or trash'");
      ROS_INFO("Is this recyclable or trash");
    }
    else if (asking_destination_mode){
      //wait for input for if object is trash or recyclable
    }
    else if (at_bin_mode){
      //wait for the human to help throw away the trash
    }

    //don't know where target is, spin
    else if (spin_mode){
      ROS_INFO("Spinning");
      cmd.angular.z = spin_direction * SPIN_SPEED;
    }
    //have the goal in sight, move to it
    else {
      ROS_INFO("Heading to goal");
      cmd.linear.x = 0.2;
      double z = 0.25 * ( (center - goal_x) / std::abs(center - goal_x) );
      cmd.angular.z = z;
    }
    
    //publish the goal and sleep till the next loop
    cmdpub_.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
