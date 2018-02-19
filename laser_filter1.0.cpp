#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/** 
 *The number of readings in the portion of the scan arc that we want to truncate from
 * either side
 */
#define DROP_SIZE 150
ros::Publisher pub; //global vars are generally considered bad manners. wrap in a new class

void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  sensor_msgs::LaserScan newMsg = *msg;

  for(int i = 0; i < DROP_SIZE; i++){
    newMsg.ranges[i] = 0.0;
  }
  int size = (sizeof(newMsg.ranges) / sizeof(newMsg.ranges[0]));
  int offset = size - DROP_SIZE;
  for(int i = 0; i < DROP_SIZE; i++){
    newMsg.ranges[offset + i] = 0.0; 
  }

  pub.publish(newMsg);
}

int main(int argc, char **argv){
  //TODO: Take filtered arc size in degrees as command line argument

  ros::init(argc, argv, "laser_filter");
  ros::NodeHandle nh;



  ros::Rate r(50); //we might use a different rate here. set to 50 to match the transform broadcasters
  
  //ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
  pub = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);

  ros::Subscriber sub = nh.subscribe("scan", 1, callback);

  ros::spin();  
}