#include <chrono>
#include <unistd.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <ros/time.h>
#include "ros/ros.h"
#include "glog/logging.h"
#include "Frame.h"

using namespace std;

void readFromROS(ros::NodeHandle &nh);

std::string lidar_topic, bag_path;
double skip = 0.0;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "AHC_Plane_Extract");
    ros::NodeHandle nh("~");

    bool screen = false;
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/fjy/Desktop/YunXiao_master/Planar_SLAM_fjy/src/AHC_Plane_Extract/Log";
    FLAGS_alsologtostderr = screen;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;


    readFromROS(nh);
    //todo 需要完善一下launch文件
    string yamlfile = "/home/fjy/Desktop/YunXiao_master/Planar_SLAM_fjy/src/AHC_Plane_Extract/config/TUM3.yaml";
    AHC_Plane::Frame ff(&nh,yamlfile);


    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}

void readFromROS(ros::NodeHandle &nh){

    string yaml_file_path = "config/TUM3.yaml";
    nh.param("yaml_file_path",yaml_file_path);
    nh.getParam("yaml_file_path",yaml_file_path);
    ROS_INFO("yaml_file_path %s", yaml_file_path.c_str());

    //nh.getParam("lidar_topic", lidar_topic);
    string RGB_topic = "/device_0/sensor_1/Color_0/image/data";
    nh.getParam("RGB_topic",RGB_topic);
    ROS_INFO("RGB_topic %s", RGB_topic.c_str());

    string depth_topic = "/device_0/sensor_0/Depth_0/image/data";
    nh.setParam("depth_topic",depth_topic);
    //nh.getParam("depth_topic",depth_topic);
    ROS_INFO("depth_topic %s", depth_topic.c_str());

//    bag_path = "/media/fjy/D/datasets/ece2-3-4.bag";
//    //bag_path = "/home/fjy/Desktop/qzj/lidar-slam/src/Run_based_segmentation/examples/example.bag";
//    //bag_path = "/media/fjy/PortableSSD/数据集/all_data.bag";
//    nh.getParam("bag_path", bag_path);
//    ROS_INFO("bag_path %s", bag_path.c_str());

}