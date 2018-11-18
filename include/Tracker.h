#ifndef PCLDATACAMP_TRACKER_H
#define PCLDATACAMP_TRACKER_H

#include <vector>
#include <tuple>
#include <algorithm>
#include <memory>
#include "RosRelated.h"
#include <pcl/kdtree/kdtree_flann.h>

class Tracker {
public:
  explicit Tracker(ros::NodeHandle &nh);

public:
  using Ptr = std::shared_ptr<Tracker>;
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  void Track(const Cloud::Ptr &centroids_in, const Cloud::Ptr &cloud_in);

private:
  ros::NodeHandle &nh_;
  ros::Publisher pub_cloud_search_;
};


#endif //PCLDATACAMP_TRACKER_H
