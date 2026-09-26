#include "yaro_check/kinematics.hpp"

#include <urdf/model.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace yaro_check
{

namespace
{

Eigen::Isometry3d toIsometry(const urdf::Pose & p)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  t.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z)
    .normalized()
    .toRotationMatrix();
  return t;
}

// Motion of one joint at position q, expressed in its own joint frame.
Eigen::Isometry3d jointMotion(const JointInfo & j, double q)
{
  Eigen::Isometry3d m = Eigen::Isometry3d::Identity();
  if (!j.actuated) {
    return m;
  }
  if (j.prismatic) {
    m.translation() = j.axis * q;
  } else {
    m.linear() = Eigen::AngleAxisd(q, j.axis).toRotationMatrix();
  }
  return m;
}

}  // namespace

Chain Chain::fromUrdf(const urdf::Model & model, const std::string & root_link, const std::string & tip_link)
{
  if (!model.getLink(root_link)) {
    throw std::runtime_error("root link not in model: " + root_link);
  }
  if (!model.getLink(tip_link)) {
    throw std::runtime_error("tip link not in model: " + tip_link);
  }

  // Walk up from the tip to the root, then reverse.
  std::vector<JointInfo> reversed;
  std::string link = tip_link;
  while (link != root_link) {
    auto l = model.getLink(link);
    auto uj = l->parent_joint;
    if (!uj) {
      throw std::runtime_error("no path from " + root_link + " to " + tip_link);
    }
    JointInfo j;
    j.name = uj->name;
    j.parent_link = uj->parent_link_name;
    j.child_link = uj->child_link_name;
    j.origin = toIsometry(uj->parent_to_joint_origin_transform);
    switch (uj->type) {
      case urdf::Joint::REVOLUTE:
        j.actuated = true;
        break;
      case urdf::Joint::CONTINUOUS:
        j.actuated = true;
        j.continuous = true;
        break;
      case urdf::Joint::PRISMATIC:
        j.actuated = true;
        j.prismatic = true;
        break;
      case urdf::Joint::FIXED:
        break;
      default:
        throw std::runtime_error("unsupported joint type in chain: " + uj->name);
    }
    if (j.actuated) {
      Eigen::Vector3d a(uj->axis.x, uj->axis.y, uj->axis.z);
      if (a.norm() < 1e-12) {
        throw std::runtime_error("zero-length axis on joint " + uj->name);
      }
      j.axis = a.normalized();
      if (uj->limits) {
        j.lower = uj->limits->lower;
        j.upper = uj->limits->upper;
        j.velocity = uj->limits->velocity;
        j.effort = uj->limits->effort;
      }
    }
    reversed.push_back(j);
    link = uj->parent_link_name;
  }

  Chain c;
  c.root_ = root_link;
  c.tip_ = tip_link;
  c.joints_.assign(reversed.rbegin(), reversed.rend());
  c.dof_ = static_cast<std::size_t>(
    std::count_if(c.joints_.begin(), c.joints_.end(), [](const JointInfo & j) {return j.actuated;}));
  return c;
}

Chain Chain::fromUrdfString(const std::string & xml, const std::string & root_link, const std::string & tip_link)
{
  urdf::Model model;
  if (!model.initString(xml)) {
    throw std::runtime_error("could not parse URDF");
  }
  return fromUrdf(model, root_link, tip_link);
}

Chain Chain::fromUrdfFile(const std::string & path, const std::string & root_link, const std::string & tip_link)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return fromUrdfString(ss.str(), root_link, tip_link);
}

std::vector<const JointInfo *> Chain::actuated() const
{
  std::vector<const JointInfo *> out;
  for (const auto & j : joints_) {
    if (j.actuated) {
      out.push_back(&j);
    }
  }
  return out;
}

Eigen::Isometry3d Chain::forward(const Eigen::VectorXd & q) const
{
  if (static_cast<std::size_t>(q.size()) != dof_) {
    throw std::invalid_argument("q has wrong size");
  }
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  Eigen::Index k = 0;
  for (const auto & j : joints_) {
    t = t * j.origin * jointMotion(j, j.actuated ? q[k] : 0.0);
    if (j.actuated) {
      ++k;
    }
  }
  return t;
}

Eigen::MatrixXd Chain::jacobian(const Eigen::VectorXd & q) const
{
  if (static_cast<std::size_t>(q.size()) != dof_) {
    throw std::invalid_argument("q has wrong size");
  }
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(dof_));
  std::vector<Eigen::Vector3d> axes, points;
  std::vector<bool> prismatic;
  axes.reserve(dof_);
  points.reserve(dof_);
  prismatic.reserve(dof_);

  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  Eigen::Index k = 0;
  for (const auto & j : joints_) {
    t = t * j.origin;  // joint frame, before its own motion
    if (j.actuated) {
      axes.push_back(t.linear() * j.axis);
      points.push_back(t.translation());
      prismatic.push_back(j.prismatic);
      t = t * jointMotion(j, q[k]);
      ++k;
    }
  }
  const Eigen::Vector3d tip = t.translation();
  for (std::size_t i = 0; i < dof_; ++i) {
    const auto col = static_cast<Eigen::Index>(i);
    if (prismatic[i]) {
      jac.block<3, 1>(0, col) = axes[i];
    } else {
      jac.block<3, 1>(0, col) = axes[i].cross(tip - points[i]);
      jac.block<3, 1>(3, col) = axes[i];
    }
  }
  return jac;
}

}  // namespace yaro_check
