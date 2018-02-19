#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 * This version of the program attempts to cure a "free(): invalid pointer" error by wrapping the
 * communication in a class
 */

/** 
 *The number of readings in the portion of the scan arc that we want to truncate from
 * either side
 */
#define DROP_SIZE 150

 /** The default size of arc to filter to, in degrees */
#define DEFAULT_FILTER_SIZE 180 


class LaserFilter{
  private:
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::Subscriber sub;

  public:
    int filteredArcDeg = DEFAULT_FILTER_SIZE;

  //constructor -- this guy needs to have the filtered arc deg and the node handle

  void communicate() {
    ros::Rate r(50); //we might use a different rate here. set to 50 to match the transform broadcasters
    pub = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
    sub = nh.subscribe("scan", 1, &LaserFilter::callback, this);
    ros::spin();
  }

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
};

int main(int argc, char **argv){
  //TODO: Take filtered arc size in degrees as command line argument

  ros::init(argc, argv, "laser_filter");

  LaserFilter laser_filter;
  laser_filter.communicate();

  //TODO: add in command line argument usage
  // if(argc == 2) {
  //   if
  // }
}