/***
 *  @brief      Project
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-06-17
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#include "controller/lqr_controller.h"

namespace laiseern
{
namespace control
{

bool LQRController::UpdatePath(const std::vector<Point> &trajectory)
{
    if (trajectory.size() < 3)
    {
        return false;
    }

    _trajectory = trajectory;
    Calheading();
    return true;
}

void LQRController::Calheading()
{
    double x_delta = 0.0;

    double y_delta = 0.0;
    double x_delta_2 = 0.0;
    double y_delta_2 = 0.0;
    for (int i = 0; i < this->_trajectory.size(); i++)
    {
        if (i == 0)
        {
            x_delta = (_trajectory[i + 1].x - _trajectory[i].x);
            y_delta = (_trajectory[i + 1].y - _trajectory[i].y);
            x_delta_2 = (_trajectory[i + 2].x - _trajectory[i + 1].x) - (_trajectory[i + 1].x - _trajectory[i].x);
            y_delta_2 = (_trajectory[i + 2].y - _trajectory[i + 1].y) - (_trajectory[i + 1].y - _trajectory[i].y);
        }
        else if (i == _trajectory.size() - 1)
        {
            x_delta = (_trajectory[i].x - _trajectory[i - 1].x);
            y_delta = (_trajectory[i].y - _trajectory[i - 1].y);
            x_delta_2 = (_trajectory[i].x - _trajectory[i - 1].x) - (_trajectory[i - 1].x - _trajectory[i - 2].x);
            y_delta_2 = (_trajectory[i].y - _trajectory[i - 1].y) - (_trajectory[i - 1].y - _trajectory[i - 2].y);
        }
        else
        {
            x_delta = 0.5 * (_trajectory[i + 1].x - _trajectory[i - 1].x);
            y_delta = 0.5 * (_trajectory[i + 1].y - _trajectory[i - 1].y);
            x_delta_2 = (_trajectory[i + 1].x - _trajectory[i].x) - (_trajectory[i].x - _trajectory[i - 1].x);
            y_delta_2 = (_trajectory[i + 1].y - _trajectory[i].y) - (_trajectory[i].y - _trajectory[i - 1].y);
        }
        //        _trajectory[i].yaw = std::atan2(y_delta, x_delta);
        //  参数方程曲率计算，曲率主要是用来对曲线的合理性进行判断的
        _trajectory[i].cur =
            (y_delta_2 * x_delta - x_delta_2 * y_delta) / std::pow((x_delta * x_delta + y_delta * y_delta), 3 / 2);
    }
}

size_t LQRController::SearchClosest()
{
    size_t closest_index = 0;
    double dist;
    double closest_distance = std::numeric_limits<double>::max();
    for (int i = 0; i < _trajectory.size(); i++)
    {
        dist = CalDistance(_trajectory[i].x, _trajectory[i].y, _cur_pose.point.x, _cur_pose.point.y);
        if (dist < closest_distance)
        {
            closest_distance = dist;
            closest_index = i;
        }
    }
    return closest_index;
}

Eigen::MatrixXd LQRController::ComputeLqr()
{
    Q << 60, 0, 0, 0, 60, 0, 0, 0, 100;

    R << 30, 0, 0, 50;

    _dt = ParamParser::GetInstance()->GetControllerParam()._dt;
    Eigen::MatrixXd A(3, 3), B(3, 2);

    Kinematic(A, B);
    double tolerance = 0.001;  // 迭代求解精度
    Eigen::MatrixXd k;
    SolveLQRProblem(A, B, Q, R, tolerance, 100, &k);
    return k;
}

void LQRController::Kinematic(Eigen::MatrixXd &A, Eigen::MatrixXd &B)
{
    double vx = _cur_pose.v;

    double ref_yaw = _cur_pose.point.yaw;
    double ref_delta =
        atan2(ParamParser::GetInstance()->GetVehicleParam()._wheel_base * this->_trajectory[_target_index].cur, 1);

    A << 1.0, 0.0, -vx * _dt * sin(ref_yaw), 0.0, 1.0, vx * _dt * cos(ref_yaw), 0.0, 0.0, 1.0;

    B << _dt * cos(ref_yaw), 0, _dt * sin(ref_yaw), 0,
        _dt * tan(ref_delta) / ParamParser::GetInstance()->GetVehicleParam()._wheel_base,
        vx * _dt / (ParamParser::GetInstance()->GetVehicleParam()._wheel_base * cos(ref_delta) * cos(ref_delta));
}

ControlCommand LQRController::CalculateControl(const State &current_state)
{
    if (_trajectory.empty())
    {
        _command.acc = 0;
        _command.steer = 0;
        return _command;
    }

    _cur_pose = current_state;
    this->desired_speed_ = _cur_pose.v;
    size_t closest_index = SearchClosest();
    _target_index = closest_index + 2;
    // 误差
    Eigen::Matrix<double, 3, 1> err;
    double yaw_err = _cur_pose.point.yaw - this->_trajectory[_target_index].yaw;

    err << _cur_pose.point.x - this->_trajectory[_target_index].x,
        _cur_pose.point.y - this->_trajectory[_target_index].y, NormalizeAngle(yaw_err);
    Eigen::MatrixXd k = ComputeLqr();
    Eigen::MatrixXd u = -k * err;
    double steering =
        u(1, 0) +
        atan2(ParamParser::GetInstance()->GetVehicleParam()._wheel_base * this->_trajectory[_target_index].cur, 1);
    NormalizeAngle(steering);
    steering = (steering * 180.0) / (70 * M_PI);

    if (isnan(steering))
    {
        steering = _previous["steering"];
    }
    else
    {
        _previous["steering"] = steering;
    }
    _command.steer = steering;
    return _command;
}

// tolerance表示迭代误差 ， max_num_iteration迭代步长N ， ptr_K 增益矩阵K
void LQRController::SolveLQRProblem(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R,
    const double tolerance, const uint max_num_iteration, MatrixXd *ptr_K)
{
    MatrixXd M = MatrixXd::Zero(Q.rows(), R.cols());
    SolveLQRProblem(A, B, Q, R, M, tolerance, max_num_iteration, ptr_K);
}

void LQRController::SolveLQRProblem(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R,
    const MatrixXd &M, const double tolerance, const uint max_num_iteration, MatrixXd *ptr_K)
{
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() ||
        R.rows() != R.cols() || R.rows() != B.cols() || M.rows() != Q.rows() || M.cols() != R.cols())
    {
        return;
    }

    MatrixXd AT = A.transpose();
    MatrixXd BT = B.transpose();
    MatrixXd MT = M.transpose();

    MatrixXd P = Q;
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();
    while (num_iteration++ < max_num_iteration && diff > tolerance)
    {
        MatrixXd P_next = AT * P * A - (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
        // check the difference between P and P_next
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
    }

    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}

}  // namespace control
}  // namespace laiseern
