#include "DatacampTut.h"

DatacampTut::DatacampTut(ros::NodeHandle &nh) : nh_(nh) {
  // Publishers
  //// Point Clouds
  pub_cloud_raw_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_raw", 1);
  pub_cloud_downsampled_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_downsampled", 1);
  pub_cloud_groundless_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_groundless", 1);
  pub_clusters_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_clusters", 1);
  pub_cluster_centroids_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_cluster_centroids", 1);

  //// Markers
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>
    ("/markers_detections", 1);

  // Subscribers
  sub_velodyne_points_ = nh.subscribe("/velodyne_points", 1,
                                     &DatacampTut::CallbackLaser, this);
}

void
DatacampTut::CallbackLaser(const sensor_msgs::PointCloud2ConstPtr &msg_cloud) {
  // Convert Point Cloud Message to PCL Point Cloud format
  Cloud::Ptr cloud_in(new Cloud);
  pcl::fromROSMsg(*msg_cloud, *cloud_in);
  RosRelated::PublishCloud(cloud_in, pub_cloud_raw_);

  // Downsample it
  Cloud::Ptr cloud_ds = PclStuff::Downsample(cloud_in, 0.1f);
  RosRelated::PublishCloud(cloud_ds, pub_cloud_downsampled_);

  // Remove ground from the downsampled point cloud
  Cloud::Ptr cloud_groundless = PclStuff::GroundRemover(cloud_ds, 0.2f);
  RosRelated::PublishCloud(cloud_groundless, pub_cloud_groundless_);

  // Apply euclidian clustering to the groundless cloud
  // And obtain centroids, cluster clouds and object sizes
  Cloud::Ptr centroids;
  Cloud::Ptr cloud_cluster;
  std::vector<float> vec_lengths_x;
  std::vector<float> vec_lengths_y;
  std::vector<float> vec_lengths_z;
  std::tie(cloud_cluster, centroids,
           vec_lengths_x, vec_lengths_y, vec_lengths_z) =
    PclStuff::MiniClusterer(cloud_groundless, 0.8, 3, 8000,
                            0.8, 0.8, 1.2,
                            0.01, 0.01, 0.01);

  RosRelated::PublishCloud(cloud_cluster, pub_clusters_);
  RosRelated::PublishCloud(centroids, pub_cluster_centroids_);

  RosRelated::PublishCylinders(centroids,
                               vec_lengths_x, vec_lengths_y, vec_lengths_z,
                               pub_markers_);
}