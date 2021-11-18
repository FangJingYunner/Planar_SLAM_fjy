//
// Created by fjy on 2021/11/16.
//

#include <chrono>
#include <unistd.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <ros/time.h>
#include "ros/ros.h"
#include "glog/logging.h"


using namespace std;

void readFromROS(ros::NodeHandle &nh);

std::string lidar_topic, bag_path;
double skip = 0.0;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "AHC_Plane_Extract");
    ros::NodeHandle nh("~");

    //nh.getParam("/use_sim_time","true");
    //nh.param<bool>("/use_sim_time","false");

//    bool screen = false;
//    google::InitGoogleLogging(argv[0]);
//    FLAGS_log_dir = "/home/fjy/Desktop/qzj/lidar-slam/src/LeGO-LOAM/Log";
//    FLAGS_alsologtostderr = screen;
//    FLAGS_colorlogtostderr = true;
//    FLAGS_log_prefix = true;
//    FLAGS_logbufsecs = 0;
//    FileManager::CreateDirectory(FLAGS_log_dir);
//
//    LOG(INFO) << "data_pretreat_node/screen: " << screen;


    readFromROS(nh);


    //std::shared_ptr<ImageProjection> IP = std::make_shared<ImageProjection>();

    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}

void readFromROS(ros::NodeHandle &nh){

    //nh.getParam("lidar_topic", lidar_topic);
    string RGB_topic = "/device_0/sensor_1/Color_0/image/data";
    nh.getParam("RGB_topic",RGB_topic);
    ROS_INFO("RGB_topic %s", RGB_topic.c_str());

    string depth_topic = "/device_0/sensor_0/Depth_0/image/data";
    nh.getParam("RGB_topic",depth_topic);
    ROS_INFO("lidar_topic %s", depth_topic.c_str());

//    bag_path = "/media/fjy/D/datasets/ece2-3-4.bag";
//    //bag_path = "/home/fjy/Desktop/qzj/lidar-slam/src/Run_based_segmentation/examples/example.bag";
//    //bag_path = "/media/fjy/PortableSSD/数据集/all_data.bag";
//    nh.getParam("bag_path", bag_path);
//    ROS_INFO("bag_path %s", bag_path.c_str());

}