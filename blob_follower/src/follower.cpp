#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;

void blobsCallBack (const cmvision::Blobs& blobsIn) //this gets the centroid of the color blob corresponding to the goal.
{
    ROS_INFO("Calling blobsCallBack");
    for (int i = 0; i < blobsIn.blob_count; i++){
      if (blobsIn.blobs[i].red == 0 && blobsIn.blobs[i].green == 0 && blobsIn.blobs[i].blue == 255){
        ROS_INFO("We got blobs");
      }
    }

}

//void PointCloud_Callback (const PointCloud::ConstPtr& cloud){
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "blobs_test");
  bool goal_trigger = 0;
  int i =0; //counter
  bool goLeft = 0;
  bool goRight = 0;
  // Create handle that will be used for both subscribing and publishing. 
  ros::NodeHandle n;
  
  //subscribe to /PointCloud2 topic 
  //ros::Subscriber PCSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);  
  ros::Subscriber PCSubscriber = n.subscribe ("/camera/depth/points", 1, cloud_cb);

  //subscribe to /blobs topic 
  ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);

  // publishing the geometry message twist message
  ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

  ROS_INFO("Program started!");
  
  // Set the loop frequency in Hz.
  ros::Rate loop_rate(10);

  geometry_msgs::Twist t;
   
  //runtime loop
  while(ros::ok()){

    ROS_INFO_THROTTLE(1, "I'm doing nothing");

    ros::spinOnce();
    loop_rate.sleep();
    velocityPublisher.publish(t);
  } 
  return 0;
}
