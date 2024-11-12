/***
*  @brief      lx_simulator
*  @author     Yong Yu
*  @version    1.0
*  @date       2023-11-28
*  @copyright  Heli Co., Ltd. All rights reserved.
***/

#ifndef LX_SIMULATOR_UTILS_HPP
#define LX_SIMULATOR_UTILS_HPP
#include <iostream>
#include <random>
#include "matplotlibcpp.h"
#include "vector"
#include "common/param_parser.h"

namespace plt = matplotlibcpp;

namespace laiseern
{
namespace planning
{
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator)
{
   //    EXPECT_TRUE(range.size() > 0u);
   // std::uniform_int_distribution 随机生产一个size_t类型的数，生成区间[0,range.size()-1]
   std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
   return range[dist(std::forward<RNG>(generator))];
}

double NormalizeAngle(float &angle);
double CalDistance(const double &x1, const double &y1, const double &x2, const double &y2);
void PlotFigure(std::vector<double> x1, std::vector<double> y1, std::vector<double> x2 = {},
   std::vector<double> y2 = {});

}  // namespace simulator
}  // namespace laiseern

#endif  // LX_SIMULATOR_UTILS_HPP
