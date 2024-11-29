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

using namespace std;


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

typedef VELOPoint PointType;
// typedef pcl::PointXYZ PointType;
pcl::PointCloud<PointType>::Ptr  l1_pcl_cloud(new  pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr  r1_pcl_cloud(new  pcl::PointCloud<PointType>());

ros::Publisher l1_cloud_pub;
ros::Publisher r1_cloud_pub;
ros::Publisher fusion_pub;
ros::Publisher imu_pub;
Eigen::Affine3f L1_R1_T;
Eigen::Matrix3f R;
Eigen::Vector3f t;
bool isGetT;

bool flipcloud = true;

void getL1ToR1_T(Eigen::Affine3f &L1_R1_t, Eigen::Matrix3f &R, Eigen::Vector3f &t)
{
    R << 0.0233267, 0.999713, -0.00605192,0.979933,-0.024062,-0.197882,-0.19797,-0.00131473,-0.980207;
    t <<  -0.00814311,-0.21899,-0.0156536;
    // Affine3f [L,t,0,1]
    L1_R1_t.linear() =  R;
    L1_R1_t.translation() = t;

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation_matrix_z;
    rotation_matrix_z = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());
    initial_guess.block<3, 3>(0, 0) = rotation_matrix_z;

    Eigen::Affine3f tempT;
    tempT.matrix() = initial_guess;
    L1_R1_t = tempT * L1_R1_t;
    
}

void LaserCloudProcess(const sensor_msgs::PointCloud2ConstPtr& l1msg,
                        const sensor_msgs::PointCloud2ConstPtr& r1msg){
    
    ros::Time timestamp  = r1msg->header.stamp;
    
    
    pcl::fromROSMsg(*l1msg,*l1_pcl_cloud);
    pcl::fromROSMsg(*r1msg,*r1_pcl_cloud);

    int plsize_l1 = l1_pcl_cloud->points.size();
    int plsize_r1 = r1_pcl_cloud->points.size();

    // auto cloud_temp =  *l1_pcl_cloud;
    // // ROS_INFO("L1 pts size: %d\n",plsize_l1);
    // cloud_temp.clear();
    pcl::transformPointCloud(*l1_pcl_cloud,*l1_pcl_cloud,L1_R1_T);
    *r1_pcl_cloud += *l1_pcl_cloud;
    

   
     
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*r1_pcl_cloud,cloud_ros);
    cloud_ros.header.stamp = ros::Time::now();
    cloud_ros.header.frame_id  =  "Hesai_link";
    fusion_pub.publish(cloud_ros);
    // std::cout << "Finish pub new cloud \n";
    l1_pcl_cloud->clear();
    r1_pcl_cloud->clear();

}

void imumsgprocess(const sensor_msgs::ImuConstPtr& imumsg)
{
    sensor_msgs::Imu imu_ros;
    imu_ros = *imumsg;
    imu_ros.header.stamp = ros::Time::now();
    imu_pub.publish(imu_ros);

}




int main(int argc, char **argv)
{
    ros::init(argc, argv,"lidarsfusionNode");
    ros::NodeHandle nh;
    std::cout << "LidarsfusionNode start -----\n";

    if(!isGetT)
    {
        getL1ToR1_T(L1_R1_T,R,t);
        ROS_INFO("Geted L1_R1_T.\n");
        isGetT = true;
    }
    fusion_pub = nh.advertise<sensor_msgs::PointCloud2>("/lio/fusion_cloud",100000);
    imu_pub =  nh.advertise<sensor_msgs::Imu>("/imu",100000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloudL1;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloudR1;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    
    // use ros message filter  mechanism
    subLaserCloudL1.subscribe(nh, "/l1_cloud", 1);
    subLaserCloudR1.subscribe(nh, "/r1_cloud", 1);
    sync_.reset(new Sync(syncPolicy(100), subLaserCloudL1, subLaserCloudR1));
    sync_->registerCallback(boost::bind(LaserCloudProcess, _1, _2));
    ros::Subscriber sub_imu = nh.subscribe("/unilidar/converted_imu", 200000,imumsgprocess);


    ros::spin();
    return 0;
}
