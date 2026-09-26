#include "yaro_check/limits.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace yaro_check
{

LimitChecker::LimitChecker(const Chain & chain, LimitCheckerConfig config)
: config_(config)
{
  if (config_.warn_margin_rad < 0.0 || config_.velocity_scale <= 0.0) {
    throw std::invalid_argument("invalid limit checker config");
  }
  for (const auto * j : chain.actuated()) {
    joints_.push_back(*j);
  }
}

std::vector<JointStatus> LimitChecker::check(
  const std::vector<std::string> & names,
  const std::vector<double> & positions,
  const std::vector<double> & velocities) const
{
  if (names.size() != positions.size()) {
    throw std::invalid_argument("names and positions differ in length");
  }
  if (!velocities.empty() && velocities.size() != names.size()) {
    throw std::invalid_argument("velocities must be empty or match names");
  }

  std::vector<JointStatus> out;
  out.reserve(joints_.size());
  for (const auto & j : joints_) {
    JointStatus s;
    s.joint = j.name;
    const auto it = std::find(names.begin(), names.end(), j.name);
    if (it == names.end()) {
      s.level = Level::kError;
      s.message = "no data";
      out.push_back(s);
      continue;
    }
    const auto idx = static_cast<std::size_t>(std::distance(names.begin(), it));
    s.position = positions[idx];
    s.velocity = velocities.empty() ? 0.0 : velocities[idx];

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4);
    if (!std::isfinite(s.position) || !std::isfinite(s.velocity)) {
      s.level = Level::kError;
      msg << "non-finite value";
    } else if (!j.continuous && (s.position < j.lower || s.position > j.upper)) {
      s.level = Level::kError;
      msg << "position " << s.position << " outside [" << j.lower << ", " << j.upper << "]";
    } else if (j.velocity > 0.0 && std::abs(s.velocity) > config_.velocity_scale * j.velocity) {
      s.level = Level::kError;
      msg << "velocity " << s.velocity << " above " << config_.velocity_scale * j.velocity;
    } else if (!j.continuous &&
      (s.position - j.lower < config_.warn_margin_rad || j.upper - s.position < config_.warn_margin_rad))
    {
      s.level = Level::kWarn;
      msg << "within " << config_.warn_margin_rad << " rad of a limit";
    } else {
      msg << "ok";
    }
    s.message = msg.str();
    out.push_back(s);
  }
  return out;
}

Level LimitChecker::worst(const std::vector<JointStatus> & s)
{
  Level w = Level::kOk;
  for (const auto & x : s) {
    w = std::max(w, x.level);
  }
  return w;
}

double LimitChecker::maxVelocityRatio(const Eigen::MatrixXd & trajectory, double dt) const
{
  if (dt <= 0.0) {
    throw std::invalid_argument("dt must be positive");
  }
  if (static_cast<std::size_t>(trajectory.cols()) != joints_.size()) {
    throw std::invalid_argument("trajectory has wrong number of columns");
  }
  double worst_ratio = 0.0;
  for (Eigen::Index r = 1; r < trajectory.rows(); ++r) {
    for (Eigen::Index c = 0; c < trajectory.cols(); ++c) {
      const double vmax = config_.velocity_scale * joints_[static_cast<std::size_t>(c)].velocity;
      if (vmax <= 0.0) {
        continue;
      }
      const double v = std::abs(trajectory(r, c) - trajectory(r - 1, c)) / dt;
      worst_ratio = std::max(worst_ratio, v / vmax);
    }
  }
  return worst_ratio;
}

}  // namespace yaro_check
