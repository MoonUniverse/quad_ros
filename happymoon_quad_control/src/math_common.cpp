#include "math_common.h"

namespace happymoon_control {

MathCommon::MathCommon() {}

Eigen::Vector3d
MathCommon::quaternionToEulerAnglesZYX(const Eigen::Quaterniond &q) {
  Eigen::Vector3d euler_angles;
  euler_angles(0) =
      atan2(2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(),
            q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  euler_angles(1) = -asin(2.0 * q.x() * q.z() - 2.0 * q.w() * q.y());
  euler_angles(2) =
      atan2(2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return euler_angles;
}

Eigen::Quaterniond
MathCommon::eulerAnglesZYXToQuaternion(const Eigen::Vector3d &euler_angles) {
  Eigen::Quaterniond q;
  double r = euler_angles(0) / 2.0;
  double p = euler_angles(1) / 2.0;
  double y = euler_angles(2) / 2.0;
  q.w() = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
  q.x() = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
  q.y() = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
  q.z() = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
  return q;
}

void MathCommon::limit(double *val, const double min, const double max) {
  if (*val > max) {
    *val = max;
  }
  if (*val < min) {
    *val = min;
  }
}

MathCommon::~MathCommon() {}

} // namespace happymoon_control
