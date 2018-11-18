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

visualization_msgs::Marker
RosRelated::points_to_arrow(geometry_msgs::Point p1,
                            geometry_msgs::Point p2,
                            int id) {
  visualization_msgs::Marker marker = visualization_msgs::Marker();
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();
  marker.ns = "markeros" + std::to_string(id);
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.2;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.a = 1.0;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  return marker;
}

visualization_msgs::MarkerArray RosRelated::ArrowSetToMarkerArray(
  std::vector<pcl::PointCloud<pcl::PointXYZ>> arrow_set, int marker_id) {

  visualization_msgs::MarkerArray markerArray = visualization_msgs::MarkerArray();
  for (unsigned long j = 0; j < arrow_set.size(); j++) {
    for (unsigned long i = 1; i < arrow_set[j].size(); i++) {
      geometry_msgs::Point point1;
      point1.x = arrow_set[j][i - 1].x;
      point1.y = arrow_set[j][i - 1].y;
      point1.z = arrow_set[j][i - 1].z;
      geometry_msgs::Point point2;
      point2.x = arrow_set[j][i].x;
      point2.y = arrow_set[j][i].y;
      point2.z = arrow_set[j][i].z;
      auto marker = points_to_arrow(point1, point2, marker_id);
      marker_id++;
      markerArray.markers.push_back(marker);
    }
  }
  return markerArray;
}

void RosRelated::PublishCloud(const Cloud::ConstPtr &cloud_in,
                              const ros::Publisher &publisher) {
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud_in, msg_cloud);
  msg_cloud.header.stamp = ros::Time::now();
  msg_cloud.header.frame_id = "velodyne";
  publisher.publish(msg_cloud);
}