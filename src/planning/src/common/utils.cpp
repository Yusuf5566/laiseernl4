/***
*  @brief      lx_simulator
*  @author     Yong Yu
*  @version    1.0
*  @date       2023-11-28
*  @copyright  Heli Co., Ltd. All rights reserved.
***/

#include "common/utils.h"

namespace laiseern
{
namespace planning
{
double NormalizeAngle(float &angle)
{
   while (angle > M_PI)
   {
       angle -= 2.0 * M_PI;
   }
   while (angle < -M_PI)
   {
       angle += 2.0 * M_PI;
   }
   angle = (angle * 180) / (70 * M_PI);
   return angle;
}

double CalDistance(const double &x1, const double &y1, const double &x2, const double &y2)
{
   double dx = x2 - x1;
   double dy = y2 - y1;
   return sqrt(dx * dx + dy * dy);
}

void PlotFigure(std::vector<double> x1, std::vector<double> y1, std::vector<double> x2, std::vector<double> y2)
{
   if (x1.size() != y1.size())
   {
       std::cout << "param1 size inequality!" << std::endl;
       size_t size = std::min(x1.size(), y1.size());
       x1.resize(size);
       y1.resize(size);
   }
   if (x2.size() != y2.size())
   {
       std::cout << "param size inequality!" << std::endl;
       size_t size = std::min(x2.size(), y2.size());
       x2.resize(size);
       y2.resize(size);
   }

   // Set the size of output image to 1200x780 pixels
   //    plt::figure_size(1200, 780);
   // Plot line from given x and y data. Color is selected automatically.
   plt::scatter(x1, y1, 1.0);
   plt::plot(x2, y2, "r.");
   // Set x-axis to interval [0,1000000]
   plt::xlim(-550, 450);
   plt::ylim(-450, 450);
   // Add graph title
   plt::title("map figure");
   plt::xlabel("x");
   plt::ylabel("y");
   // Save the image (file format is determined by the extension)
   std::string path = "./map.svg";
   //    ParamParser::ParamParse("carla_bridge_path", path);
   //    std::cout << "save path: " << path << std::endl;
   plt::save(path);
}

}  // namespace planning
}  // namespace laiseern