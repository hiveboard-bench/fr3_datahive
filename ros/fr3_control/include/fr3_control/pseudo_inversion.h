// Author: Enrico Corvaglia
// https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_controllers/include/utils/pseudo_inversion.h
// File provided under public domain
//
// Damped (Tikhonov) pseudo-inverse via SVD. Carried over unchanged from
// serl_franka_controllers so the nullspace projector behaves identically; the
// damping term lambda keeps it finite near kinematic singularities.
//
// Note this allocates: JacobiSVD does its own heap work, so it runs inside the
// 1 kHz thread exactly as it does upstream. That is inherited behaviour, not a
// choice this fork endorses -- if the loop is ever seen overrunning, this is the
// first thing to replace with a fixed-size decomposition.

#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace fr3_control
{

  inline void pseudoInverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_, bool damped = true)
  {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_; // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
      S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
  }

} // namespace fr3_control
