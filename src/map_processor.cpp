#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class MapProcessor
{
public:
    MapProcessor() :
        m_occupancyGridWidth(100),  // 
        m_occupancyGridHeight(100), // Set grid height (in cells)
        m_tfListener(m_tfBuffer)
    {
        m_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("/occup_grid_map_pb", 1);
        m_sub = m_nh.subscribe("/output", 1, &MapProcessor::MapCallBack, this);
    }

private:
    void MapCallBack(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        nav_msgs::OccupancyGrid occupancyGrid;
        occupancyGrid.header.frame_id = "map";
        occupancyGrid.header.stamp = ros::Time::now();
        occupancyGrid.info.width = m_occupancyGridWidth;
        occupancyGrid.info.height = m_occupancyGridHeight;
        occupancyGrid.info.resolution = 0.1;
        occupancyGrid.info.origin.position.x = -m_occupancyGridWidth / 2.0 * occupancyGrid.info.resolution;
        occupancyGrid.info.origin.position.y = -m_occupancyGridHeight / 2.0 * occupancyGrid.info.resolution;
        occupancyGrid.info.origin.orientation.w = 1.0;
        occupancyGrid.data.resize(m_occupancyGridWidth * m_occupancyGridHeight, -1); 

        for (const auto& point : cloud->points)
        {
            int x = static_cast<int>((point.x - occupancyGrid.info.origin.position.x) / occupancyGrid.info.resolution);
            int y = static_cast<int>((point.y - occupancyGrid.info.origin.position.y) / occupancyGrid.info.resolution);

            if (x >= 0 && x < occupancyGrid.info.width && y >= 0 && y < occupancyGrid.info.height) {
                int index = y * occupancyGrid.info.width + x;
                occupancyGrid.data[index] = 100;
            }
        }

        m_pub.publish(occupancyGrid);
    }

    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;

    int m_occupancyGridWidth;
    int m_occupancyGridHeight;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_processor");
    MapProcessor mp;
    ros::spin();
    return 0;
}
