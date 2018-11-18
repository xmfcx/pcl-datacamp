#ifndef PCLDATACAMP_DATACAMPTUT_H
#define PCLDATACAMP_DATACAMPTUT_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PclStuff.h"
#include "RosRelated.h"
#include "Tracker.h"

class DatacampTut {
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  explicit DatacampTut(ros::NodeHandle &nh);

private:
  ros::Subscriber sub_velodyne_points_;

  ros::Publisher pub_cloud_raw_;
  ros::Publisher pub_cloud_downsampled_;
  ros::Publisher pub_cloud_groundless_;
  ros::Publisher pub_clusters_;
  ros::Publisher pub_cluster_centroids_;
  ros::Publisher pub_markers_;

  ros::NodeHandle& nh_;

  Tracker::Ptr tracker_;

  void CallbackLaser(const sensor_msgs::PointCloud2ConstPtr &msg_cloud);
};


#endif //PCLDATACAMP_DATACAMPTUT_H
