
#include "Frame.h"


namespace AHC_Plane{

    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    int Frame::Frame_Count=0;

    Frame::Frame(ros::NodeHandle* nh,const string yamlfile):Fnh(nh){
        //todo 读取RGB-D的yaml配置文件
        //Config::SetParameterFile(yamlfile);

        cv::FileStorage fsSettings(yamlfile.c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << yamlfile << endl;
            exit(-1);
        }

        fx = fsSettings["Camera.fx"];
        fy = fsSettings["Camera.fx"];
        cx = fsSettings["Camera.cx"];
        cy = fsSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);//相机内参14jiang p99
        K.at<float>(0, 0) = fx;//旋转
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;//平移
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fsSettings["Camera.k1"];//设置畸变参数
        DistCoef.at<float>(1) = fsSettings["Camera.k2"];
        DistCoef.at<float>(2) = fsSettings["Camera.p1"];
        DistCoef.at<float>(3) = fsSettings["Camera.p2"];
        const float k3 = fsSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mDepthMapFactor = fsSettings["DepthMapFactor"];
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;

        //
        int img_width = fsSettings["Camera.width"];
        int img_height = fsSettings["Camera.height"];


        string depth_topic;
        Fnh->getParam("depth_topic",depth_topic);
        sub_depth = Fnh->subscribe<sensor_msgs::Image>(depth_topic, 1, &Frame::DepthHandler, this);

    }

    void Frame::DepthHandler(const sensor_msgs::ImageConstPtr & Depth_row_image){
        auto cv_ptr =  cv_bridge::toCvCopy(*Depth_row_image, sensor_msgs::image_encodings::TYPE_16UC1);
        Depth_img = cv_ptr->image;

        cv::Mat tempMat = Depth_img(cv::Rect(0, 0, 840, 480));

        cv::Mat depth;
        if (mDepthMapFactor != 1 || Depth_img.type() != CV_32FC1) {
            tempMat.convertTo(depth, CV_32FC1, mDepthMapFactor);
        }
        ComputePlanes(depth, tempMat, RGB_img, mK);

    }

    void Frame::ComputePlanes(const cv::Mat &imDepth, const cv::Mat &Depth, const cv::Mat &imRGB, cv::Mat K) {
        planeDetector.readDepthImage(Depth, K);
        planeDetector.runPlaneDetection();
//        LOG(INFO)<<"Frame_Count"<<Frame_Count++;
        vector<int> max_N_ind;
        for (int i = 0; i < planeDetector.plane_num_; i++) {
            auto extractedPlane = planeDetector.plane_filter.extractedPlanes[i];
            max_N_ind.push_back(extractedPlane->N);
//            auto &indices = planeDetector.plane_vertices_[i];
//            PointCloud::Ptr inputCloud(new PointCloud());
//            for (int j : indices) {
//                PointT p;
//                p.x = (float) planeDetector.cloud.vertices[j][0];
//                p.y = (float) planeDetector.cloud.vertices[j][1];
//                p.z = (float) planeDetector.cloud.vertices[j][2];
//
//                inputCloud->points.push_back(p);
//            }
//            pcl::VoxelGrid<PointT> voxel;
//            voxel.setLeafSize(0.05, 0.05, 0.05);
//
//            PointCloud::Ptr coarseCloud(new PointCloud());
//            voxel.setInputCloud(inputCloud);
//            voxel.filter(*coarseCloud);
//
//            cv::Mat coef = (cv::Mat_<float>(4, 1) << nx, ny, nz, d);
//            bool valid = MaxPointDistanceFromPlane(coef, coarseCloud);
//
//            if (!valid) {
//                continue;
//            }
//
//            mvPlanePoints.push_back(*coarseCloud);
//
//            mvPlaneCoefficients.push_back(coef);
        }
        auto biggest = max_element(max_N_ind.begin(),max_N_ind.end());
        auto biggest_ind = distance(max_N_ind.begin(),biggest);
        auto extractedPlane = planeDetector.plane_filter.extractedPlanes[biggest_ind];
        double nx = extractedPlane->normal[0];
        double ny = extractedPlane->normal[1];
        double nz = extractedPlane->normal[2];
        double cx = extractedPlane->center[0];
        double cy = extractedPlane->center[1];
        double cz = extractedPlane->center[2];
        float d = (float) -(nx * cx + ny * cy + nz * cz);

        //todo 写一个法向量和距离的消息发布
        //todo 之后可能会因为精度要求用下面的代码计算平面法向量
//        LOG(INFO)<<"normal:"<<nx<<ny<<nz;
//        LOG(INFO)<<"d:"<<d<<endl;
//        cout<<"normal:"<<nx<<" "<<ny<<" "<<nz<<endl;
//        cout<<"d:"<<d<<endl;

//        std::vector<SurfaceNormal> surfaceNormals;
//
//        PointCloud::Ptr inputCloud( new PointCloud() );
//        for (int m=0; m<imDepth.rows; m+=3)
//        {
//            for (int n=0; n<imDepth.cols; n+=3)
//            {
//                float d = imDepth.ptr<float>(m)[n];
//                PointT p;
//                p.z = d;
//                //cout << "depth:" << d<<endl;
//                p.x = ( n - cx) * p.z / fx;
//                p.y = ( m - cy) * p.z / fy;
//
//                inputCloud->points.push_back(p);
//            }
//        }
//        inputCloud->height = ceil(imDepth.rows/3.0);
//        inputCloud->width = ceil(imDepth.cols/3.0);
//
//        //compute normals
//        pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
//        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
//        ne.setMaxDepthChangeFactor(0.05f);
//        ne.setNormalSmoothingSize(10.0f);
//        ne.setInputCloud(inputCloud);
//        //计算特征值
//
//        if (inputCloud->size()== 0)
//        {
//            PCL_ERROR ("Could not estimate a planar model for the given initial plane.\n");
//            return;
//        }
//        ne.compute(*cloud_normals);
//
//        for ( int m=0; m<inputCloud->height; m+=1 ) {
//            if(m%2==0) continue;
//            for (int n = 0; n < inputCloud->width; n+=1) {
//                pcl::Normal normal = cloud_normals->at(n, m);
//                SurfaceNormal surfaceNormal;
//                if(n%2==0) continue;
//                surfaceNormal.normal.x = normal.normal_x;
//                surfaceNormal.normal.y = normal.normal_y;
//                surfaceNormal.normal.z = normal.normal_z;
//
//                pcl::PointXYZRGB point = inputCloud->at(n, m);
//                surfaceNormal.cameraPosition.x = point.x;
//                surfaceNormal.cameraPosition.y = point.y;
//                surfaceNormal.cameraPosition.z = point.z;
//                surfaceNormal.FramePosition.x = n*3;
//                surfaceNormal.FramePosition.y = m*3;
//
//                surfaceNormals.push_back(surfaceNormal);
//            }
//        }
//
//        vSurfaceNormal = surfaceNormals;

    }

    bool Frame::MaxPointDistanceFromPlane(cv::Mat &plane, PointCloud::Ptr pointCloud) {
        auto disTh = Config::Get<double>("Plane.DistanceThreshold");
        bool erased = false;
//        double max = -1;
        double threshold = 0.04;
        int i = 0;
        auto &points = pointCloud->points;
//        std::cout << "points before: " << points.size() << std::endl;
        for (auto &p : points) {
            double absDis = abs(plane.at<float>(0) * p.x +
                                plane.at<float>(1) * p.y +
                                plane.at<float>(2) * p.z +
                                plane.at<float>(3));

            if (absDis > disTh)
                return false;
            i++;
        }

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(disTh);

        seg.setInputCloud(pointCloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given initial plane.\n");
            return false;
        }

        float oldVal = plane.at<float>(3);
        float newVal = coefficients->values[3];

        cv::Mat oldPlane = plane.clone();


        plane.at<float>(0) = coefficients->values[0];
        plane.at<float>(1) = coefficients->values[1];
        plane.at<float>(2) = coefficients->values[2];
        plane.at<float>(3) = coefficients->values[3];

        if ((newVal < 0 && oldVal > 0) || (newVal > 0 && oldVal < 0)) {
            plane = -plane;
//                double dotProduct = plane.dot(oldPlane) / sqrt(plane.dot(plane) * oldPlane.dot(oldPlane));
//                std::cout << "Flipped plane: " << plane.t() << std::endl;
//                std::cout << "Flip plane: " << dotProduct << std::endl;
        }
//        }

        return true;
    }
}