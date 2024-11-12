/***
 *  @brief      planning
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-04-02
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_POINT_TYPES_H
#define LAISEERNL4_POINT_TYPES_H

#include "string"
#include "vector"
#include <glog/logging.h>

namespace laiseern
{
namespace planning
{
struct Point
{
    double x;
    double y;
    double z;
    double yaw;  // direction on the x-y plane
    double cur;
};

struct State
{
    Point point;
    double v;
    double vx;
    double vy;
    double acc;
    double acc_x;
    double acc_y;
    double jerk;
    double speed_limit;
};

enum class LaneChange : uint8_t
{
    None = 0x00,   // 00
    Right = 0x01,  // 01
    Left = 0x02,   // 10
    Both = 0x03    // 11
};

struct ReferenceLinePoint
{
    LaneChange lane_change = LaneChange::None;
    Point point{};
    double cur = 0;
    double s = 0;
};

struct TrajectoryPoint
{
    // path point
    State state{};
    double v;
    double v_x;
    double v_y;
    // linear acceleration
    double a;
    double a_x;
    double a_y;

    // relative time from beginning of the trajectory
    double relative_time{};

    // accumulated distance from beginning of the path
    double s;
    // curvature on the x-y planning
    double kappa;
    // derivative of kappa w.r.t s.
    double dkappa;
    // derivative of derivative of kappa w.r.t s.
    double ddkappa;
    // The lane ID where the path point is on
    std::string lane_id;
};

struct STGraphPoint
{
    double s{};
    double t{};
    TrajectoryPoint init_point;
    double total_cost = std::numeric_limits<double>::infinity();
    STGraphPoint* pre_point = nullptr;
    double reference_cost = 0.0;
    double obstacle_cost = 0.0;
    std::uint32_t index_s = 0;
    std::uint32_t index_t = 0;
};

struct FrenetPoint
{
    State state;
    double s;
    double l;

    double s_d;
    double l_d;
    double s_d_d;
    double l_d_d;
    double s_d_d_d;
    double l_d_d_d;

    double l_ds;
    double l_d_ds;
    double l_d_d_ds;
    double ds;
};

struct DpPathPoint
{
    LaneChange lane_change = LaneChange::None;
    FrenetPoint _frenet_point;
    double _cost;
    DpPathPoint* _pre_point;
    int _pre_row;
};

struct SLBoundary
{
    double _start_s = 0.0;
    double _end_s = 0.0;
    double _start_l = 0.0;
    double _end_l = 0.0;
};

struct STBoundary
{
    std::vector<std::pair<double, double>> _lower_points;  // s,t
    std::vector<std::pair<double, double>> _upper_points;
};

struct ObstaclePoint
{
    enum class ObstacleType
    {
        Unknown = 0,
        Vehicle = 1,
        Pedestrian = 2,
        Cyclist = 3,
        Static = 4
    };
    FrenetPoint _frenet_point{};
    double _x_rad{};
    double _y_rad{};
    std::vector<FrenetPoint> _collision_box;
    SLBoundary _sl_boundary;
    STBoundary _st_boundary;
    ObstacleType _obstacle_type = ObstacleType::Unknown;
};

struct SpeedPoint
{
    double speed_limit{};
    double _s;
    double _t;
    // speed (m/s)
    double _v;
    // acceleration (m/s^2)
    double _a;
    // jerk (m/s^3)
    double _da;
};

}  // namespace planning
}  // namespace laiseern

#endif  // LAISEERNL4_POINT_TYPES_H
