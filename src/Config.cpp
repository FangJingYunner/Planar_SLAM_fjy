//
// Created by fishmarch on 19-5-24.
//

#include "Config.h"

namespace AHC_Plane{

    shared_ptr<Config> Config::mConfig = nullptr;

    void Config::SetParameterFile( const std::string& filename )
    {
        if ( mConfig == nullptr )
            mConfig = shared_ptr<Config>(new Config);
        mConfig->mFile = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
        if ( !mConfig->mFile.isOpened())
        {
            std::cerr<<"parameter file "<< filename <<" does not exist."<<std::endl;
            mConfig->mFile.release();
            return;
        }
    }

    Config::~Config()
    {
        if ( mFile.isOpened() )
            mFile.release();
    }


}