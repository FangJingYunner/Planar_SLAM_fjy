#ifndef FRAME_H
#define FRAME_H

#include "Config.h"
#include <ros/ros.h>
#include "PlaneExtractor.h"
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/integral_image_normal.h>

#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"
#include "glog/logging.h"

namespace AHC_Plane {
    using namespace std;

//    #define FRAME_GRID_ROWS 48
//    #define FRAME_GRID_COLS 64
    class SurfaceNormal {
    public:
        cv::Point3f normal;
        cv::Point3f cameraPosition;
        cv::Point2i FramePosition;

        SurfaceNormal() {}
    };

    class Frame
    {
    public:
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud <PointT> PointCloud;

        static int Frame_Count;
        Frame(ros::NodeHandle *nh,const string yamlfile);

        bool MaxPointDistanceFromPlane(cv::Mat &plane, PointCloud::Ptr pointCloud);

//        void RGBHandler(const sensor_msgs::ImageConstPtr & RGB_row_image);
        void DepthHandler(const sensor_msgs::ImageConstPtr & Depth_row_image);


    public:
        ros::NodeHandle* Fnh;
        ros::Subscriber sub_RGB;
        ros::Subscriber sub_depth;
        double mTimeStamp;
        std::vector<SurfaceNormal> vSurfaceNormal;
        cv::Mat mK;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;
        cv::Mat RGB_img,Depth_img;
        float mDepthMapFactor;
        int img_width;
        int img_height;


        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;

        // Number of KeyPoints.
        int N;
        int NL;
        bool dealWithLine;
        float blurNumber;


        long unsigned int mnId;

        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        std::vector<float> mvScaleFactors;
        std::vector<float> mvInvScaleFactors;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;


        std::vector<cv::Mat> mvPlaneCoefficients;

        PlaneDetection planeDetector;


    private:

        void ComputePlanes(const cv::Mat &imDepth, const cv::Mat &depth, const cv::Mat &imGrey, cv::Mat K);
    };
}
#endif