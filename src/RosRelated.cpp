#include "RosRelated.h"

void RosRelated::PublishCylinders(const Cloud::Ptr &centroids,
                                  const std::vector<float> &vector_length_x,
                                  const std::vector<float> &vector_length_y,
                                  const std::vector<float> &vector_length_z,
                                  const ros::Publisher &publisher) {
  visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();

  for (int i = 0; i < 100; ++i) {
    visualization_msgs::Marker marker = visualization_msgs::Marker();
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "markerss" + std::to_string(i);
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    if (i >= centroids->points.size()) {
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.color.a = 0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker_array.markers.push_back(marker);
      continue;
    }

    marker.pose.position.x = centroids->points[i].x;
    marker.pose.position.y = centroids->points[i].y;
    marker.pose.position.z = centroids->points[i].z;

    marker.scale.x = vector_length_x[i];
    marker.scale.y = vector_length_y[i];
    marker.scale.z = vector_length_z[i];


    marker.color.a = 0.4; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker_array.markers.push_back(marker);
  }
  publisher.publish(marker_array);
}


void RosRelated::PublishCloud(const Cloud::ConstPtr &cloud_in,
                              const ros::Publisher &publisher) {
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud_in, msg_cloud);
  msg_cloud.header.stamp = ros::Time::now();
  msg_cloud.header.frame_id = "velodyne";
  publisher.publish(msg_cloud);
}