//
// Created by cbt on 18-5-30.
//

#ifndef VISUALODOMETRY_CONFIG_H
#define VISUALODOMETRY_CONFIG_H

#include "myslam/common_include.h"

namespace myslam
{

class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}

public:
    ~Config();

    static void setParameterFile( const std::string &fileName );

    template < typename T >
    static T get ( const std::string &key)
    {
        return T( Config::config_->file_[key] );
    }

};

}


#endif //VISUALODOMETRY_CONFIG_H
