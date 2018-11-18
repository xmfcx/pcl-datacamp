#include "Tracker.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using KdTree = pcl::KdTreeFLANN<pcl::PointXYZI>;

Tracker::Tracker(ros::NodeHandle &nh) : nh_(nh) {
  centroids_old.reset(new Cloud);
  pub_markers_arrows = nh_.advertise<visualization_msgs::MarkerArray>
    ("/markers_arrows", 1);

}

void Tracker::Track(const Cloud::Ptr &centroids_in) {
  if (first_run) {
    centroids_old = centroids_in;
    for (const auto &point : centroids_in->points) {
      Cloud::Ptr tracked_set(new Cloud);
      tracked_set->points.push_back(point);
      arrow_sets.push_back(tracked_set);
    }
    return;
  }

  KdTree::Ptr kdtree_centroids(new KdTree);
  kdtree_centroids->setInputCloud(centroids_in);

  for (auto &arrow_set:arrow_sets) {
    // Search around end points of each arrow set
    for (const auto &searchPoint:arrow_set->points) {
      // Neighbors within radius search
      std::vector<int> indices;
      std::vector<float> distances_squared;
      float radius = 0.5;

//      std::cout << "Neighbors within radius search at ("
//                << searchPoint.x
//                << " " << searchPoint.y
//                << " " << searchPoint.z
//                << ") with radius=" << radius << std::endl;

      if (kdtree_centroids->radiusSearch(searchPoint, radius, indices,
                                         distances_squared) > 0) {
        // Print search results if found
//        for (size_t i = 0; i < indices.size(); ++i) {
//          std::cout << "    " << centroids_in->points[indices[i]].x
//                    << " " << centroids_in->points[indices[i]].y
//                    << " " << centroids_in->points[indices[i]].z
//                    << " (squared distance: " << distances_squared[i] << ")"
//                    << std::endl;
//        }

        // Add it to the current tracked set
        int ind_closest = indices[0];
        const auto& point_closest =  centroids_in->points[ind_closest];
        arrow_set->points.push_back(point_closest);
      }
    }
  }




}


