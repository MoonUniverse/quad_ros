#ifndef MATH_COMMON_H
#define MATH_COMMON_H
#include <eigen3/Eigen/Dense>

namespace happymoon_control {

class MathCommon {
public:
  MathCommon();
  ~MathCommon();

  Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond &q);
  Eigen::Quaterniond
  eulerAnglesZYXToQuaternion(const Eigen::Vector3d &euler_angles);

  void limit(double *val, const double min, const double max);

private:
};

} // namespace happymoon_control

#endif
