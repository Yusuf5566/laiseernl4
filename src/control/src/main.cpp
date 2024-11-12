/***
 *  @brief      control
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-12-01
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/
#include "control_component.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");

    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_ERROR);
    std::string control_log_path;
    laiseern::control::ParamParser::ParamParse("control_log_path", control_log_path);
    std::string info_log_path = control_log_path + "info.txt";
    std::string warning_log_path = control_log_path + "warning.txt";
    std::string error_log_path = control_log_path + "error.txt";
    google::SetLogDestination(google::GLOG_INFO, info_log_path.c_str());
    google::SetLogDestination(google::GLOG_WARNING, warning_log_path.c_str());
    google::SetLogDestination(google::GLOG_ERROR, error_log_path.c_str());
    FLAGS_logtostderr = 0;
    FLAGS_colorlogtostderr = true;

    laiseern::control::ControlComponent control_component;
    control_component.Init();
    control_component.Process();

    return 0;
}