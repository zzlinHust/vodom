//
// Created by cbt on 18-5-30.
//

#include "config.h"


namespace myslam
{

Config::~Config()
{
    if( file_.isOpened() )
        file_.release();
}


void Config::setParameterFile( const std::string &fileName )
{

    if(config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);


    config_->file_ = cv::FileStorage( fileName.c_str(), cv::FileStorage::READ );
    if(!config_->file_.isOpened())
    {
        std::cerr << "parameter file " << fileName << " doesn't exist." << std::endl;
        config_->file_.release();
        return ;
    }
}

std::shared_ptr<Config> Config::config_ = nullptr;



}