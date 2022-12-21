#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Subscriber sub_pandar_passthrough;
ros::Publisher pub_cluster;

void ClusterProcess(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

    ec.setClusterTolerance(3);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZI pt = cloud->points[*pit];
            pt.intensity - j;
            cloud_cluster->points.push_back(pt);
        }
        j++;
    }
    cloud_cluster->header.frame_id = "pos";
    pub_cluster.publish(cloud_cluster);
}

void pandarCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*cloud);

    ClusterProcess(cloud);
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"pandar_process");
    ros::NodeHandle nh;

    sub_pandar_passthrough = nh.subscribe("pandar_passthrough", 10, &pandarCallback);
    pub_cluster = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("pandar_cluster",10);

    ros::spin();

    return 0;
}