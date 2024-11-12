/***
 *  @brief      Project
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-06-17
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_LQR_CONTROLLER_H
#define LAISEERNL4_LQR_CONTROLLER_H

#include "controller.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>
#include <unordered_map>
#include "common/control_point_types.h"

using namespace Eigen;

namespace laiseern
{
namespace control
{

class LQRController : public Controller
{
public:
    LQRController() = default;

    bool UpdatePath(const std::vector<Point> &trajectory) override;
    ControlCommand CalculateControl(const State &current_state) override;

private:
    void Calheading();
    void Kinematic(Eigen::MatrixXd& A, Eigen::MatrixXd& B);

    Eigen::MatrixXd ComputeLqr();
    size_t SearchClosest();
    void SolveLQRProblem(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R, double tolerance,
        uint max_num_iteration, MatrixXd *ptr_K);
    void SolveLQRProblem(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R, const MatrixXd &M,
        double tolerance, uint max_num_iteration, MatrixXd *ptr_K);

private:
    Eigen::Matrix<double, 3, 3> Q;                      // Q矩阵
    Eigen::Matrix<double, 2, 2> R;                      // R矩阵
    std::unordered_map<std::string, double> _previous;  // 用于临时存储一些上一循环周期的变量
    double desired_speed_{};
    State _cur_pose{};
    size_t _target_index{};
    double _dt = 0.1;
    ControlCommand _command{};
};

RegisterClass(Controller, LQRController);

}  // namespace control
}  // namespace laiseern

#endif  // LAISEERNL4_LQR_CONTROLLER_H
