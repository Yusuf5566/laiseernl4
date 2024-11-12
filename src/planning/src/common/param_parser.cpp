/***
*  @brief      Project
*  @author     Yong Yu
*  @version    1.0
*  @date       2024-04-29
*  @copyright  Heli Co., Ltd. All rights reserved.
***/

#include "common/param_parser.h"

namespace laiseern
{
namespace planning
{
ParamParser* ParamParser::_control_param_parser;

ParamParser::ParamParser()
{
   Init();
}

void ParamParser::Init()
{

}

ParamParser* ParamParser::GetInstance()
{
   if (_control_param_parser == nullptr)
   {
       _control_param_parser = new ParamParser();
   }
   return _control_param_parser;
}

}  // namespace control
}  // namespace laiseern
