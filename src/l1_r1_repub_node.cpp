/*****
 * @author: Hong Yuanqian, @github:piluohong
 * @brief: Use ros message filter mechanism to synchronize L1 and R1 PointCLoud
 * *******/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <Eigen/Core>

// ros message filter princple
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std;


struct  HESAIPoint{
    PCL_ADD_POINT4D;
    float intensity; // intensity
    double timestamp;
    uint16_t ring;
    // uint8_t ring; 
    // int line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(HESAIPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp)
                                 (uint16_t, ring, ring))

struct  VELOPoint{
    PCL_ADD_POINT4D;
    float intensity; // intensity
    float time;
    uint16_t ring;
    // uint8_t ring; 
    // int line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VELOPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (float, time, time)
                                 (uint16_t, ring, ring))

// typedef PointXYZIT PointType;
typedef pcl::PointXYZ PointType;
pcl::PointCloud<PointType>::Ptr  l1_pcl_cloud(new  pcl::PointCloud<PointType>());
pcl::PointCloud<VELOPoint>::Ptr  l1_pcl_cloud_(new  pcl::PointCloud<VELOPoint>());
// pcl::PointCloud<PointType>::Ptr  cloud_saved(new  pcl::PointCloud<PointType>());
pcl::PointCloud<HESAIPoint>::Ptr  r1_pcl_cloud(new  pcl::PointCloud<HESAIPoint>());
pcl::PointCloud<VELOPoint>::Ptr  r1_pcl_cloud_(new  pcl::PointCloud<VELOPoint>());

ros::Publisher l1_cloud_pub;
ros::Publisher r1_cloud_pub;
ros::Publisher imu_pub;
Eigen::Affine3f L1_R1_T;
Eigen::Matrix3f R;
Eigen::Vector3f t;



void l1msgprocess(const sensor_msgs::PointCloud2ConstPtr& l1msg)
{
    pcl::fromROSMsg(*l1msg,*l1_pcl_cloud);
    sensor_msgs::PointCloud2 cloud_ros;
    double timeheader = l1msg->header.stamp.toSec();
    int ptsize = l1_pcl_cloud->size();
    for(size_t i = 0; i < ptsize;i++)
    {
        VELOPoint pt;
        pt.x = l1_pcl_cloud->points[i].x;
        pt.y = l1_pcl_cloud->points[i].y;
        pt.z = l1_pcl_cloud->points[i].z;
        pt.intensity = 255;
        pt.ring = 1;
        pt.time = ((1/15)/ptsize) * i;
        l1_pcl_cloud_->points.push_back(pt);
    }
    pcl::toROSMsg(*l1_pcl_cloud_,cloud_ros);
    cloud_ros.header.stamp = ros::Time::now();
    cloud_ros.header.frame_id  =  "pandarXT-16";
    l1_cloud_pub.publish(cloud_ros);
    // std::cout << "l1msg converted \n";
    l1_pcl_cloud_->clear();
    l1_pcl_cloud->clear();
    // *cloud_saved  += *l1_pcl_cloud;


}
void r1msgprocess(const sensor_msgs::PointCloud2ConstPtr& r1msg)
{
    pcl::fromROSMsg(*r1msg,*r1_pcl_cloud);
    sensor_msgs::PointCloud2 cloud_ros;
    double timeheader = r1msg->header.stamp.toSec();
    int ptsize = r1_pcl_cloud->size();
    for(size_t i = 0; i < ptsize;i++)
    {
        VELOPoint pt;
        pt.x = r1_pcl_cloud->points[i].x;
        pt.y = r1_pcl_cloud->points[i].y;
        pt.z = r1_pcl_cloud->points[i].z;
        pt.intensity = r1_pcl_cloud->points[i].intensity;
        pt.ring = r1_pcl_cloud->points[i].ring;
        pt.time = (r1_pcl_cloud->points[i].timestamp - timeheader); //s
        r1_pcl_cloud_->points.push_back(pt);
    }
    pcl::toROSMsg(*r1_pcl_cloud_,cloud_ros);
    cloud_ros.header.stamp = ros::Time::now();
    cloud_ros.header.frame_id  =  "pandarXT-16";
    r1_cloud_pub.publish(cloud_ros);
    // std::cout << "r1msg converted \n";
    r1_pcl_cloud->clear();
    r1_pcl_cloud_->clear();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv,"l1_r1_repub_node");
    ros::NodeHandle nh;
    std::cout << "l1_r1_repub_node start -----\n";
    
    // time repub
    l1_cloud_pub =  nh.advertise<sensor_msgs::PointCloud2>("/l1_cloud",100000);
    r1_cloud_pub =  nh.advertise<sensor_msgs::PointCloud2>("/r1_cloud",100000);
   
    ros::Subscriber sub_l1 = nh.subscribe("/unilidar/converted_pointcloud", 200000, l1msgprocess);
    ros::Subscriber sub_r1 = nh.subscribe("/hesai/pandar", 200000, r1msgprocess);
   

    ros::spin();
    // pcl::io::savePCDFileBinary("/home/u20/hyq_rl/data/PCD/L1/l1.pcd",*cloud_saved);
    return 0;
}
