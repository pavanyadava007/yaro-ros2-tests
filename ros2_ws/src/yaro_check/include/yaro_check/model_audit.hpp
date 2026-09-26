// Static checks on a robot description: things that break simulators, planners
// or ROS tools long before anything moves. Every check is a named, testable rule.
#pragma once

#include <string>
#include <vector>

namespace urdf
{
class Model;
}

namespace yaro_check
{

struct Finding
{
  std::string rule;     // e.g. "inertia.positive_definite"
  std::string element;  // link or joint name
  bool error{false};    // false = warning
  std::string detail;
};

struct AuditSummary
{
  std::string robot;
  std::size_t links{0};
  std::size_t joints{0};
  std::size_t actuated{0};
  double total_mass{0.0};
  std::vector<Finding> findings;
  std::size_t errors() const;
  std::size_t warnings() const;
};

// Rules:
//  structure.single_root        exactly one root link
//  structure.root_inertia       the root link carries <inertial> (KDL ignores it; warning)
//  joint.limits_ordered         lower < upper for revolute / prismatic joints
//  joint.velocity_positive      velocity limit > 0
//  joint.effort_positive        effort limit > 0
//  joint.axis_unit              axis has unit length (URDF does not normalise it for every consumer)
//  link.mass_positive           links with <inertial> have mass > 0
//  inertia.positive_definite    all principal moments > 0
//  inertia.triangle             principal moments satisfy I1 + I2 >= I3 (physically realisable body)
//  mesh.uri_scheme              mesh filenames use package:// or file:// (warning otherwise)
AuditSummary audit(const urdf::Model & model);
AuditSummary auditString(const std::string & xml);

}  // namespace yaro_check
