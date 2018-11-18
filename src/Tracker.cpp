#include "Tracker.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using KdTree = pcl::KdTreeFLANN<pcl::PointXYZI>;

Tracker::Tracker(ros::NodeHandle &nh) : nh_(nh) {
  pub_cloud_search_ = nh_.advertise<sensor_msgs::PointCloud2>
    ("/cloud_search", 1);
}

void
Tracker::Track(const Cloud::Ptr &centroids_in, const Cloud::Ptr &cloud_in) {
  KdTree::Ptr kdtree_centroids(new KdTree);
  kdtree_centroids->setInputCloud(cloud_in);

  Cloud::Ptr cloud_radius(new Cloud);

  for (const auto &centroid:centroids_in->points) {
    // Neighbors within radius search
    std::vector<int> indices;
    std::vector<float> distances_squared;
    float radius = 1;


    if (kdtree_centroids->radiusSearch(centroid, radius, indices,
                                       distances_squared) > 0) {
      // Print search results if found
//      for (size_t i = 0; i < indices.size(); ++i) {
//        std::cout << "    " << centroids_in->points[indices[i]].x
//                  << " " << centroids_in->points[indices[i]].y
//                  << " " << centroids_in->points[indices[i]].z
//                  << " (squared distance: " << distances_squared[i] << ")"
//                  << std::endl;
//      }

      std::cout << "indices: " << indices.size() << std::endl;
      // Add it to the radius cloud
      for (const int &index : indices) {
        const auto &point_search = cloud_in->points[index];
        cloud_radius->points.push_back(point_search);
      }
//      int ind_closest = indices[0];
//      const auto &point_closest = centroids_in->points[ind_closest];
    }
  }

  RosRelated::PublishCloud(cloud_radius, pub_cloud_search_);
}


