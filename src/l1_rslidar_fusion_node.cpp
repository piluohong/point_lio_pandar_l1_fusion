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

// class lidarsFusion{
//     public:
        

//     public:
//         lidarsFusion(){
            
//         };
//         ~lidarsFusion();
void getL1ToR1_T(Eigen::Affine3f &L1_R1_t, Eigen::Matrix3f &R, Eigen::Vector3f &t)
{
    R << 0.0187884,0.999817,-0.00433973,0.979138,-0.0192767,-0.202289,-0.202336,-0.000448603,-0.979318;
    t <<  -0.0294266,-0.241189,-0.021259;
    // Affine3f [L,t,0,1]
    L1_R1_t.linear() =  R;
    L1_R1_t.translation() = t;
    
}

void LaserCloudProcess(const sensor_msgs::PointCloud2ConstPtr& l1msg,
                        const sensor_msgs::PointCloud2ConstPtr& r1msg){
    
    ros::Time timestamp  = r1msg->header.stamp;
    
    
    pcl::fromROSMsg(*l1msg,*l1_pcl_cloud);
    pcl::fromROSMsg(*r1msg,*r1_pcl_cloud);

    int plsize_l1 = l1_pcl_cloud->points.size();
    int plsize_r1 = r1_pcl_cloud->points.size();

    auto cloud_temp =  *l1_pcl_cloud;
    // ROS_INFO("L1 pts size: %d\n",plsize_l1);
    // cloud_temp.clear();
    pcl::transformPointCloud(*l1_pcl_cloud,cloud_temp,L1_R1_T);

    *r1_pcl_cloud += cloud_temp;
     
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*r1_pcl_cloud,cloud_ros);
    cloud_ros.header.stamp = ros::Time::now();
    cloud_ros.header.frame_id  =  "Hesai_link";
    fusion_pub.publish(cloud_ros);
    // std::cout << "Finish pub new cloud \n";

}

void imumsgprocess(const sensor_msgs::ImuConstPtr& imumsg)
{
    sensor_msgs::Imu imu_ros;
    imu_ros = *imumsg;
    imu_ros.header.stamp = ros::Time::now();
    imu_pub.publish(imu_ros);

}



// bool  fromROSMsg(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//     return false;
// }

    // private:

    

// };



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
