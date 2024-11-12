/***
*  @brief      Project
*  @author     Yong Yu
*  @version    1.0
*  @date       2024-04-29
*  @copyright  Heli Co., Ltd. All rights reserved.
***/

#ifndef LAISEERNL4_PARAM_PARSER_H
#define LAISEERNL4_PARAM_PARSER_H
#include "ros/ros.h"
#include "planning_param.h"

namespace laiseern
{
namespace planning
{
class ParamParser
{
public:
public:
   static ParamParser* GetInstance();
   template <class T>
   static T ParamParse(const std::string name, T& value)
   {
       ros::NodeHandle nh("~");
       nh.getParam(name, value);
       return value;
   }

private:
   void Init();
   ParamParser();
   static ParamParser* _control_param_parser;
};

}  // namespace planning
}  // namespace laiseern
#endif  // LAISEERNL4_PARAM_PARSER_H
