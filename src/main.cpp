#include <ros/ros.h>
#include <ros/spinner.h>
#include "DatacampTut.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "clusterer");
  ros::NodeHandle nh;
  DatacampTut datacamp_tut(nh);

  ros::AsyncSpinner spinner(5);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
