#ifndef PATHFINDING_SERVICE_NODELET_HPP
#define PATHFINDING_SERVICE_NODELET_HPP

#include <cmath>
#include <mutex>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ares_control/RRT.hpp>
#include <ares_control/GridUtils.hpp>
#include <ares_control/CheckPath.h>
#include <ares_control/GetPath.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ares_control
{
  class PathfindingServiceNodelet : public nodelet::Nodelet
  {
    public:
    
    virtual void onInit()
    {
      ros::NodeHandle &nh = getMTNodeHandle();

      float occupancy_grid_resolution_level, occupancy_grid_length;
      nh.getParam("occupancy_grid/resolution", occupancy_grid_resolution_level);
      nh.getParam("occupancy_grid/length", occupancy_grid_length);      
      m_occupancy_grid = boost::make_shared<nav_msgs::OccupancyGrid>();
      m_occupancy_grid->info.resolution = std::pow(2, occupancy_grid_resolution_level);
      float number_of_cells = occupancy_grid_length/m_occupancy_grid->info.resolution; 
      m_occupancy_grid->info.width = number_of_cells;
      m_occupancy_grid->info.height = number_of_cells;
      m_occupancy_grid->data.resize(number_of_cells*number_of_cells, -1);

      m_tf_buffer = std::make_shared<tf2_ros::Buffer>();
      m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
      m_sub = nh.subscribe("point_cloud", 1, &PathfindingServiceNodelet::pointCloudCallback, this);
      m_get_path_server = nh.advertiseService("get_path", &PathfindingServiceNodelet::handlePathRequest, this);
      m_check_path_server = nh.advertiseService("check_path", &PathfindingServiceNodelet::handleCheckCollisionCourseRequest, this);
    }

    private:
    ros::Subscriber m_sub;
    ros::ServiceServer m_get_path_server;
    ros::ServiceServer m_check_path_server;
    std::mutex m_occupancy_grid_mutex;
    std::shared_ptr <tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr <tf2_ros::TransformListener> m_tf_listener;
    boost::shared_ptr<nav_msgs::OccupancyGrid> m_occupancy_grid;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
    {
      sensor_msgs::PointCloud2Ptr cloud_global_ros (new sensor_msgs::PointCloud2);
      pcl::PCLPointCloud2Ptr cloud_global_pcl (new pcl::PCLPointCloud2);
      pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;

      pcl_ros::transformPointCloud("map", *cloud_in, *cloud_global_ros, *m_tf_buffer);
      pcl_conversions::toPCL(*cloud_global_ros, *cloud_global_pcl);
      voxel_filter.setInputCloud(cloud_global_pcl);
      voxel_filter.setLeafSize(0.05, 0.05, 0.1);
      voxel_filter.filter(*cloud_global_pcl);
      voxel_filter.setInputCloud(cloud_global_pcl);
      voxel_filter.setLeafSize(0.03, 0.03, 10.0);
      voxel_filter.setMinimumPointsNumberPerVoxel(2);
      voxel_filter.filter(*cloud_global_pcl);
      voxel_filter.setInputCloud(cloud_global_pcl);
      voxel_filter.setLeafSize(0.11, 0.11, 10.0);
      voxel_filter.setMinimumPointsNumberPerVoxel(2);
      voxel_filter.filter(*cloud_global_pcl);

      std::lock_guard<std::mutex> lock(m_occupancy_grid_mutex);
      m_occupancy_grid->header.stamp = ros::Time::now();
      m_occupancy_grid->info.map_load_time = ros::Time::now();
      m_occupancy_grid->data.assign(m_occupancy_grid->data.size(), -1);
      updateOccupancyGrid(*cloud_global_pcl, *m_occupancy_grid);
    }

    bool handlePathRequest(ares_control::GetPath::Request &req, ares_control::GetPath::Response &res)
    {
      std::lock_guard<std::mutex> lock(m_occupancy_grid_mutex);
      /* Calculate and return a path from the position of the rover to the goal. */
      return true;
    }
    
    bool handleCheckCollisionCourseRequest(ares_control::CheckPath::Request &req, ares_control::CheckPath::Response &res)
    {
      return true;
    } 

    void updateOccupancyGrid(const pcl::PCLPointCloud2 &cloud, nav_msgs::OccupancyGrid &grid)
    {
      std::lock_guard<std::mutex> lock(m_occupancy_grid_mutex);
      /* Draw OccupancyGrid from X-Y coordinates of the input PointCloud. */
      return;
    }

    geometry_msgs::PoseStamped getRoverPose()
    {
      geometry_msgs::PoseStamped pose;
      m_tf_buffer->transform(geometry_msgs::PoseStamped(), pose, "map", ros::Time(0), "base_link");
      return pose;
    }

  }; // class PathfindingServiceNodelet
} // namespace ares_control

#endif // PATHFINDING_SERVICE_NODELET_HPP