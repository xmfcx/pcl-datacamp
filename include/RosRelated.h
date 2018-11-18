#ifndef PCLDATACAMP_ROSRELATED_H
#define PCLDATACAMP_ROSRELATED_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

class RosRelated {
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  static void PublishCylinders(const Cloud::Ptr &centroids,
                               const std::vector<float> &vector_length_x,
                               const std::vector<float> &vector_length_y,
                               const std::vector<float> &vector_length_z,
                               const ros::Publisher &publisher);

  static void PublishCloud(const Cloud::ConstPtr &cloud_in,
                           const ros::Publisher &publisher);
};


#endif //PCLDATACAMP_ROSRELATED_H
