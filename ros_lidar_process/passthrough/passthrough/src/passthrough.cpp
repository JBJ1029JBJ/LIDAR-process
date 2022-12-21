#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transformation_from_correspondences.h>

ros::Subscriber sub_pandar;
ros::Publisher pub_pandar_passthrough;

void PassthroughCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    ros::Time t = msg->header.stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg,*cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PassThrough <pcl::PointXYZI> pass;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1,3);

    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 100);

    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5, 5);

    pass.filter(*cloud_filter);

    sensor_msgs::PointCloud2 ptr_msg;
    ptr_msg.header.stamp = t;
    ptr_msg.header.frame_id = "pos";

    pcl::toROSMsg(*cloud_filter, ptr_msg);

    pub_pandar_passthrough.publish(ptr_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"pandar_passthrough");
    ros::NodeHandle nh;

    sub_pandar = nh.subscribe("output", 10, &PassthroughCallback);
    pub_pandar_passthrough = nh.advertise<sensor_msgs::PointCloud2>("pandar_passthrough",10);

    ros::spin();

    return 0;
}