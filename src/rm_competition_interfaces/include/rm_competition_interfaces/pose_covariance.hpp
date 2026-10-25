#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <Eigen/Eigenvalues>

namespace rm_competition_interfaces
{
// AUD-015 full pose-covariance semantics, shared by A and both Z boundaries.
// All-zero/missing and rank-deficient PSD matrices are legal; no noise floor.
inline bool validatePoseCovariance(const std::array<double, 36> & covariance,
  std::string * reason = nullptr)
{
  const auto reject = [](std::string * output, const char * text) {
      if (output) {*output = text;}
      return false;
    };
  if (!std::all_of(covariance.begin(), covariance.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    return reject(reason, "non-finite covariance");
  }
  // Zero unobserved dimensions are legal; arbitrary cross-covariance is not.
  Eigen::Matrix<double, 6, 6> matrix;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      matrix(row, col) = covariance[row * 6 + col];
    }
    if (matrix(row, row) < 0.0) {
      return reject(reason, "negative covariance");
    }
  }
  // Normalize by each dimension's standard deviation, so a large unused
  // variance cannot hide a negative eigenvalue in the navigation block.
  Eigen::Matrix<double, 6, 6> normalized;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      if (matrix(row, row) == 0.0 || matrix(col, col) == 0.0) {
        if (matrix(row, col) != 0.0) {return reject(reason, "non-PSD covariance");}
        normalized(row, col) = 0.0;
      } else {
        normalized(row, col) = matrix(row, col) /
          std::sqrt(matrix(row, row)) / std::sqrt(matrix(col, col));
      }
    }
  }
  constexpr double tolerance = 1.0e-10;
  if (!normalized.allFinite() || normalized.cwiseAbs().maxCoeff() > 1.0 + tolerance) {
    return reject(reason, "non-PSD covariance");
  }
  if ((normalized - normalized.transpose()).cwiseAbs().maxCoeff() > tolerance) {
    return reject(reason, "asymmetric covariance");
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
    0.5 * (normalized + normalized.transpose()), Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -tolerance) {
    return reject(reason, "non-PSD covariance");
  }
  if (reason) {reason->clear();}
  return true;
}
}  // namespace rm_competition_interfaces
