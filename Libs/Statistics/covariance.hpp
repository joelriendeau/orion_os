#pragma once

#include "Projections/projections.hpp"
#include "LinearAlgebra/linear_algebra.hpp"
#include "PatchedEigen/patched_eigen.hpp"

namespace stats {

namespace covariance {

template <typename CovarBase>
void global_to_local(const ellipsoids::ellipsoid& ell, const coordinates::earth_centered& coord, const Eigen::Matrix<CovarBase, 3, 3>& covar, Eigen::Matrix<CovarBase, 3, 3>& result)
{
    Eigen::Matrix<CovarBase, 3, 3> rotation;
    coordinates::ecef_to_local_rot(ell, coord, rotation);
    result = rotation * covar;
    result *= rotation.transpose(); // not elegant but does not compile on one line
}

template <typename CovarBase>
void local_to_global(const ellipsoids::ellipsoid& ell, const coordinates::earth_centered& coord, const Eigen::Matrix<CovarBase, 3, 3>& covar, Eigen::Matrix<CovarBase, 3, 3>& result)
{
    Eigen::Matrix<CovarBase, 3, 3> rotation;
    coordinates::ecef_to_local_rot(ell, coord, rotation);
    result = rotation.transpose() * covar * rotation;
}

template <typename CovarBase>
void drms_horizontal(const Eigen::Matrix<CovarBase, 3, 3>& covar, CovarBase& drms)
{
    Eigen::Matrix<std::complex<CovarBase>, 2, 1> eigen_vals = covar.block<2,2>(0,0).eigenvalues();
    drms = std::sqrt(std::abs(eigen_vals(0)) + std::abs(eigen_vals(1)));
}

template <typename CovarBase>
void drms_vertical(const Eigen::Matrix<CovarBase, 3, 3>& covar, CovarBase& drms)
{
    drms = std::sqrt(covar(2,2));
}

template <typename CovarBase>
void mrse_3d(const Eigen::Matrix<CovarBase, 3, 3>& covar, CovarBase& mrse) // like drms but in 3d
{
    Eigen::Matrix<std::complex<CovarBase>, 3, 1> eigen_vals = covar.eigenvalues();
    mrse = std::sqrt(std::abs(eigen_vals(0)) + std::abs(eigen_vals(1)) + std::abs(eigen_vals(2)));
}

template <typename CovarBase>
void cep_horizontal(const Eigen::Matrix<CovarBase, 3, 3>& covar, CovarBase& cep)
{
    Eigen::Matrix<std::complex<CovarBase>, 2, 1> eigen_vals = covar.block<2,2>(0,0).eigenvalues();
    // This constant was obtained from several sources. A thorough derivation may be available in
    // "Fundamentals of High Accuracy Inertial Navigation", from Averil B. Chatfield.
    // It is an approximation valid when the larger standard deviation is no greater than 3 times the smaller one.
    cep = static_cast<CovarBase>(0.589) * std::sqrt(std::abs(eigen_vals(0)) + std::sqrt(std::abs(eigen_vals(1)))); // can be optimized : sqrt of abs is like ^(1/4)
}

template <typename CovarBase>
void sep_3d(const Eigen::Matrix<CovarBase, 3, 3>& covar, CovarBase& sep)
{
    Eigen::Matrix<std::complex<CovarBase>, 3, 1> eigen_vals = covar.eigenvalues();
    // This constant was obtained from Novatel's APN-029 Rev.1 accuracy document.
    sep = static_cast<CovarBase>(0.51) * std::sqrt(std::abs(eigen_vals(0)) + std::sqrt(std::abs(eigen_vals(1))) + std::sqrt(std::abs(eigen_vals(2)))); // can be optimized : sqrt of abs is like ^(1/4)
}

template <typename CovarBase>
void sep_3d_99(const Eigen::Matrix<CovarBase, 3, 3>& covar, CovarBase& sep_99) // 99% SEP
{
    Eigen::Matrix<std::complex<CovarBase>, 3, 1> eigen_vals = covar.eigenvalues();
    // This constant was obtained from Novatel's APN-029 Rev.1 accuracy document.
    sep_99 = static_cast<CovarBase>(1.122) * std::sqrt(std::abs(eigen_vals(0)) + std::sqrt(std::abs(eigen_vals(1))) + std::sqrt(std::abs(eigen_vals(2)))); // can be optimized : sqrt of abs is like ^(1/4)
}

}

}