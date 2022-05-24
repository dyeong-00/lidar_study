#include "lidarstudy.h"

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
    //Create pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*inputcloud, *cloud); //rosmsg에서 pointXYZI로 변환

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (1.0);
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    std::cout << "Number of clusters is " << clusterIndices.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) //point 하나하나에 접근
        {
            pcl::PointXYZ pt = cloud->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(clusterId + 1); //다른 clusters에 대해 다른 intensity를 넣어주기위해

            TotalCloud.push_back(pt2);
        }
        ++clusterId;
    }


    // publish할 sensor_msgs::PointCloud2 선언
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(TotalCloud, output);

    output.header.frame_id = "velodyne";
    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("토픽명", 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("cluster", 1);

    ros::spin();

    return 0;
}
