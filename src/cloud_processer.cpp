#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class MyCloudProcessor{

public:
    MyCloudProcessor() :
        m_tfListener(m_tfBuffer)
        {
          m_pub = m_nh.advertise<sensor_msgs::PointCloud2>("output", 1);
          m_sub = m_nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 1, &MyCloudProcessor::cloudCallback, this);
        }
private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input){
        ROS_INFO("Used");
        pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*input, *pcl_cloud);

        pcl::PCLPointCloud2::Ptr cloud_voxel(new pcl::PCLPointCloud2());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2::Ptr cloud_transform2(new pcl::PCLPointCloud2()); // Declaration of cloud_transform2

        pcl::VoxelGrid<pcl::PCLPointCloud2> vds_pc2;
        pcl::VoxelGrid<pcl::PointXYZ> vds_xyz;
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pcl::PassThrough<pcl::PointXYZ> pass_z;

        vds_pc2.setInputCloud(pcl_cloud);
        vds_pc2.setLeafSize(0.01, 0.01, 0.01);
        vds_pc2.filter(*cloud_voxel);

        pcl::fromPCLPointCloud2(*cloud_voxel, *cloud_transform);

        pass.setInputCloud(cloud_transform);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.2, 10.0);
        pass.filter(*cloud_transform);

        pass_y.setInputCloud(cloud_transform);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-10.0, 10.0);
        pass_y.filter(*cloud_transform);

        // Passthrough filter on z-axis
        pass_z.setInputCloud(cloud_transform);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-2.0, 2.0);
        pass_z.filter(*cloud_transform);

        pcl_ros::transformPointCloud("map", *cloud_transform, *cloud_transform1, m_tfBuffer);

        for (std::size_t idx = 0; idx < cloud_transform1->size(); ++idx) {
            (*cloud_transform1)[idx].z = 0;
        }

        vds_xyz.setInputCloud(cloud_transform1);
        vds_xyz.setLeafSize(0.01, 0.01, 0.01);
        vds_xyz.filter(*cloud_transform1);

        pcl::toPCLPointCloud2(*cloud_transform1, *cloud_transform2);

        cloud_transform2->header.frame_id = "map";
        m_pub.publish(*cloud_transform2);
    }

    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_processor");
    MyCloudProcessor cp;
    ros::spin();
    return 0;
}
