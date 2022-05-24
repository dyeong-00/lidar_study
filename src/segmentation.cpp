#include "lidarstudy.h"

ros::Publisher pub1;
ros::Publisher pub2;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
    //Create pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planecloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*inputcloud, *cloud); //rosmsg에서 pointXYZI로 변환

    //segment plane
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.2);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    //separate clouds
    for(int index : inliers->indices)       
    {
        planecloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstcloud);

    // publish할 sensor_msgs::PointCloud2 선언
    sensor_msgs::PointCloud2 cloud_obst;
    sensor_msgs::PointCloud2 cloud_plane;

    pcl::toROSMsg(*obstcloud, cloud_obst);
    pcl::toROSMsg(*planecloud, cloud_plane);

    cloud_obst.header.frame_id = "velodyne";
    cloud_plane.header.frame_id = "velodyne";
    pub1.publish(cloud_obst);
    pub2.publish(cloud_plane);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("토픽명", 1, cloud_cb);

    pub1 = nh.advertise<sensor_msgs::PointCloud2>("cloud_obst", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);


    ros::spin();

    return 0;
}
