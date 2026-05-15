#pragma once

#include <Eigen/Dense>

namespace franka_impedance_controller {

inline Eigen::Matrix<double, 6, 7> pseudoInverse(
    const Eigen::Ref<const Eigen::Matrix<double, 7, 6>>& matrix,
    bool damped = true) {
  const double lambda = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svd(matrix,
                                                    Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 6, 7> singular_values_inverse = Eigen::Matrix<double, 6, 7>::Zero();

  for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
    const double singular_value = svd.singularValues()(i);
    singular_values_inverse(i, i) =
        singular_value / (singular_value * singular_value + lambda * lambda);
  }

  Eigen::Matrix<double, 6, 7> v_times_singular_values_inverse;
  v_times_singular_values_inverse.noalias() = svd.matrixV() * singular_values_inverse;

  Eigen::Matrix<double, 6, 7> pseudo_inverse;
  pseudo_inverse.noalias() = v_times_singular_values_inverse * svd.matrixU().transpose();
  return pseudo_inverse;
}

}  // namespace franka_impedance_controller
