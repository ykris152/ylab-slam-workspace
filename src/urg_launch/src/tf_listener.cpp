#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

tf::TransformListener* listener;
ros::Publisher pub;

void hokuyo3dCallback(const sensor_msgs::PointCloud &pointcloud){
    sensor_msgs::PointCloud tfed_pointcloud;
    sensor_msgs::PointCloud2 tfed_pointcloud2;
    try{
        listener->transformPointCloud("base_link", pointcloud, tfed_pointcloud);
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("Failed transform.");
    }
    convertPointCloudToPointCloud2(tfed_pointcloud, tfed_pointcloud2);
    pub.publish(tfed_pointcloud2);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("hokuyo3d/hokuyo_cloud", 5, hokuyo3dCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("reverse_cloud", 5);
    tf::TransformListener lr(ros::Duration(100));
    listener = &lr;
    ros::spin();

    return 0;
}
