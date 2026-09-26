// Joint position and velocity limit checks against the URDF limits.
// Pure logic, no ROS types, so it can be unit tested without a running node.
#pragma once

#include "yaro_check/kinematics.hpp"

#include <string>
#include <vector>

namespace yaro_check
{

enum class Level { kOk = 0, kWarn = 1, kError = 2 };

struct JointStatus
{
  std::string joint;
  Level level{Level::kOk};
  std::string message;
  double position{0.0};
  double velocity{0.0};
};

struct LimitCheckerConfig
{
  double warn_margin_rad{0.05};    // WARN when closer than this to a position limit
  double velocity_scale{1.0};      // allowed fraction of the URDF velocity limit (e.g. 0.25 for a reduced-speed mode)
};

class LimitChecker
{
public:
  LimitChecker(const Chain & chain, LimitCheckerConfig config);

  // names/positions/velocities follow the sensor_msgs/JointState convention: velocities may be empty.
  // Joints of the chain that are missing from `names` are reported as ERROR ("no data").
  std::vector<JointStatus> check(
    const std::vector<std::string> & names,
    const std::vector<double> & positions,
    const std::vector<double> & velocities) const;

  // Worst level in a status list.
  static Level worst(const std::vector<JointStatus> & s);

  // Largest |dq/dt| / (velocity_scale * v_max) over a sampled trajectory (rows = samples, cols = dof).
  // A value above 1.0 means the trajectory violates the velocity limit somewhere.
  double maxVelocityRatio(const Eigen::MatrixXd & trajectory, double dt) const;

private:
  std::vector<JointInfo> joints_;  // actuated joints only
  LimitCheckerConfig config_;
};

}  // namespace yaro_check
