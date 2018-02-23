/*
Name: trash_bot.cpp
Author: Jeremy Smith
Date: 

Description: 
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
bool done_mode;
bool spin_mode;
bool wander_mode;
bool avoid_mode;
bool obstacle_detected;
bool found_trash_mode;
bool pickup_mode;
bool move_trash_mode;

////////

int avoid_direction; //which way to turn to avoid an obstacle. + for left, - for right
int spin_direction; //direction to spin when searching. Default 1, but set to be opposite of avoid_direction
int spin_mode_enter_time;

int done_criteria = 80000; //Area of color that must be present to decide we are at the goal
double center = 320.0; //Center of the screen
double goal_x = 0.0; //Average x position of color blobs

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher cmdpub_; //publisher for movement commands

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
  done_mode = false;
  enterSpinMode();
  avoid_mode = false;
  obstacle_detected = false;

  //TODO: for neatness, init all the ther states to false

  spin_direction = 1;
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

    //check if we are at the target
    if (color_area > done_criteria){
      done_mode = true;
    }
    //get the average x position of all COLOR blobs
    else if (color_blob_count > 0 && color_area > color_area_thresh){
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
  float z_thresh = 0.5;

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

  if (pickup_mode && numValid > 400 && z > 0.5 && z < 0.8){
    pickup_mode = false;
    move_trash_mode = true;
    system("rosrun sound_play say.py 'Thank you'");
    return;
  }

  //obstacle detected!
  if (numValid > 2000){
    ROS_INFO_THROTTLE(1, "Centroid detected at x: %f z: %f, with %d points", x, z, numValid);

    obstacle_detected = true;

    if (spin_mode){
      //if in spin mode that's fine, keep spinning
    }
    //otherwise, enter avoid_mode
    else{
      avoid_mode = true;
      avoid_direction = x / std::abs(x);
      spin_direction = -1 * avoid_direction;
    }
  }
  //no obstacle detected, 
  else{ 
    obstacle_detected = false;
  }
  
}

/*
Callback for heard voices. Recognizes words in the following patterns:

*/
void voiceCallBack(const std_msgs::String& msg){

  const char* temp = msg.data.c_str();
  ROS_INFO_THROTTLE(1, "Heard the following:\n%s", temp);
}

void foundTrash(){
  ROS_INFO("About to play first audio");
  system("rosrun sound_play say.py 'I found some trash'");
  ROS_INFO("About to play second audio");
  system("rosrun sound_play say.py 'Will you help me pick it up'");
  ROS_INFO("Done speaking");

  found_trash_mode = false;
  pickup_mode = true;

}


int main (int argc, char** argv) {
  ros::init(argc, argv, "blobs_test");

  // Create handle that will be used for both subscribing and publishing. 
  ros::NodeHandle n;
  
  //system("rosrun blob_follower blob_follower BLUE");
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

    
    //at the goal, stop
    if (done_mode){
      break;
    }
    //found trash, ask for help picking it up
    else if (found_trash_mode){
      foundTrash();
    }
    //wait until person helps out
    else if (pickup_mode){
      //TODO: write code to make sure that you can't get out of pickup mode until someone helps out
    }
    //received trash
    else if (move_trash_mode){
      break;
    }

    //there is an obstacle, avoid it
    else if (avoid_mode){
      //if in avoid mode and we see the obstacle, spin 90 degrees away from it
      if (obstacle_detected){
        cmd.angular.z = avoid_direction * 2; //spin 90 degrees
      }
      //if in avoid mode and we don't see the obstacle, move forward for 2 seconds or until
      //a new obstacle is detected
      else{
        int t0 = ros::Time::now().toSec();
        while (!obstacle_detected && ros::Time::now().toSec() - t0 < 2){
          geometry_msgs::Twist avoid_cmd;
          avoid_cmd.linear.x = 0.2;
          cmdpub_.publish(avoid_cmd);
          ros::spinOnce();
          loop_rate.sleep();
          avoid_mode = false;
          enterSpinMode();
        }
      }
    }
    //don't know where target is, spin
    else if (spin_mode){
      int t0 = ros::Time::now().toSec();
      if (t0 - spin_mode_enter_time < 10){
        ROS_INFO("Spinning");
        cmd.angular.z = spin_direction * SPIN_SPEED;
      }
      //wander forwards for 3 seconds if we've been spinning for too long
      else{
        ROS_INFO("Wandering!");
        spin_mode = false;
        wander_mode = true;
        while (!obstacle_detected && ros::Time::now().toSec() - t0 < 3){
          geometry_msgs::Twist wander_cmd;
          wander_cmd.linear.x = 0.2;
          cmdpub_.publish(wander_cmd);
          ros::spinOnce();
          loop_rate.sleep();
        }
        wander_mode = false;
        enterSpinMode();
      }
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
  ROS_INFO("At the target!");
  return 0;
}
