#include "lidarstudy.h"

ros::Publisher pub;
ros::Publisher pub2;

struct Box
{
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

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
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI> eachCloud;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) //point 하나하나에 접근
        {
            pcl::PointXYZ pt = cloud->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(clusterId + 1); //다른 clusters에 대해 다른 intensity를 넣어주기위해
            
            eachCloud.push_back(pt2);
            TotalCloud.push_back(pt2);
        }

        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(eachCloud, minPoint, maxPoint);
        Box box;
        box.x_min = minPoint.x;
        box.y_min = minPoint.y;
        box.z_min = minPoint.z;
        box.x_max = maxPoint.x;
        box.y_max = maxPoint.y;
        box.z_max = maxPoint.z;

        float xsize = std::abs(box.x_max - box.x_min);
        float ysize = std::abs(box.y_max - box.y_min);
        float zsize = std::abs(box.z_max - box.z_min);
        float volume = xsize * ysize * zsize;

        float loc_x = (box.x_max - box.x_min)/2 + box.x_min;
        float loc_y = (box.y_max - box.y_min)/2 + box.y_min;
        float loc_z = (box.z_max - box.z_min)/2 + box.z_min;

        if (0.05 < xsize && xsize < 0.6 && 0.15 < ysize && ysize < 0.6 && 0.15<zsize && zsize<3 && volume > 0.1)
        {
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time();
            marker.ns = clusterIndices.size();
            marker.id = clusterId;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = (box.x_min+box.x_max)/2;
            marker.pose.position.y = (box.y_min+box.y_max)/2;
            marker.pose.position.z = (box.z_min+box.z_max)/2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = xsize;
            marker.scale.y = ysize;
            marker.scale.z = zsize;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);
          }

        ++clusterId;
        float distance = std::sqrt(loc_x * loc_x + loc_y *loc_y);
        distance = std::sqrt(distance * distance + loc_z * loc_z);
        std::cout << "DISTANCE FROM BOX : " << distance << std::endl; 
    }


    // publish할 sensor_msgs::PointCloud2 선언
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(TotalCloud, output);

    output.header.frame_id = "velodyne";
    pub.publish(output);
    pub2.publish(marker_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "boundingbox");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("cluster", 1);
    pub = nh.advertise<visualization_msgs::MarkerArray>("boundingbox", 1);
    ros::spin();

    return 0;
}
