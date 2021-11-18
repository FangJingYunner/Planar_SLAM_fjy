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

//#include <pcl/common/transforms.h>
//#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>

namespace AHC_Plane {
    using namespace std;



    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64
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

        Frame();

        // Copy constructor.
        Frame(const string yamlfile);

        // Constructor for RGB-D cameras.
//        Frame(const cv::Mat &imRGB, const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const float &depthMapFactor);

        // Extract ORB on the image. 0 for left image and 1 for right image.
        //void ExtractORB(int flag, const cv::Mat &im);

        // extract line feature
        //void ExtractLSD(const cv::Mat &im, const cv::Mat &depth,cv::Mat K);

        //void isLineGood(const cv::Mat &imGray,  const cv::Mat &imDepth,cv::Mat K);

        //void lineDescriptorMAD( std::vector<std::vector<cv::DMatch>> matches, double &nn_mad, double &nn12_mad) const;

        // Compute Bag of Words representation.
        //void ComputeBoW();

        // Set the camera pose.
        //void SetPose(cv::Mat Tcw);

        // Computes rotation, translation and camera center matrices from the camera pose.

        // Returns the camera center.
        bool MaxPointDistanceFromPlane(cv::Mat &plane, PointCloud::Ptr pointCloud);

        // Returns inverse of rotation

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking

    public:

        // Frame timestamp.
        double mTimeStamp;
        std::vector<SurfaceNormal> vSurfaceNormal;
        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;

        // Stereo baseline multiplied by fx.


        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;

        // Number of KeyPoints.
        int N;
        int NL;
        bool dealWithLine;
        float blurNumber;
        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.


        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;
        std::vector<float>  mvDepthLine;
        // Bag of Words Vector structures.
        std::vector<PointCloud> mvPlanePoints;

        // ORB descriptor, each row associated to a keypoint.
        cv::Mat mDescriptors, mDescriptorsRight;

        // MapPoints associated to keypoints, NULL pointer if no association.

        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier;




        //2D endpoints, 3D LinesPC

        cv::Mat mLdesc;
        std::vector<Eigen::Vector3d> mvKeyLineFunctions;
        std::vector<bool> mvbLineOutlier;

        vector<cv::Point3d> mv3DLineforMap;
        //vector<Vector6d > mvLines3D;
        //vector<FrameLine> mVF3DLines;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.


        // Camera pose.
        cv::Mat mTcw;
        cv::Mat mTwc;

        // Rotation, translation and camera center
        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mRwc;
        cv::Mat mOw; //==mtwc

        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId;

        // Reference Keyframe.

        // Scale pyramid info.
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        std::vector<float> mvScaleFactors;
        std::vector<float> mvInvScaleFactors;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;

        // Undistorted Image Bounds (computed once).
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

        std::vector<cv::Mat> mvPlaneCoefficients;

        // Flag to identify outlier planes new planes.
        std::vector<bool> mvbPlaneOutlier;
        std::vector<bool> mvbParPlaneOutlier;
        std::vector<bool> mvbVerPlaneOutlier;
        int mnPlaneNum;
        bool mbNewPlane; // used to determine a keyframe

        PlaneDetection planeDetector;


    private:

        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).

        // Computes image bounds for the undistorted image (called in the constructor).

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).

        void ComputePlanes(const cv::Mat &imDepth, const cv::Mat &depth, const cv::Mat &imGrey, cv::Mat K);
    };
}
#endif