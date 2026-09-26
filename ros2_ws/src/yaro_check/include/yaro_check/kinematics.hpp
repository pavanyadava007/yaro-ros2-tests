// Serial-chain kinematics built from a URDF, with no dependency on KDL.
// KDL is used only in the tests, as an independent reference.
#pragma once

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace urdf
{
class Model;
}

namespace yaro_check
{

struct JointInfo
{
  std::string name;
  std::string parent_link;
  std::string child_link;
  bool actuated{false};  // revolute / continuous / prismatic
  bool prismatic{false};
  bool continuous{false};
  Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};  // parent link -> joint frame at q = 0
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};            // in the joint frame, unit length
  double lower{0.0};
  double upper{0.0};
  double velocity{0.0};
  double effort{0.0};
};

class Chain
{
public:
  // Walks from root_link to tip_link. Throws std::runtime_error if the path does not exist.
  static Chain fromUrdf(const urdf::Model & model, const std::string & root_link, const std::string & tip_link);
  static Chain fromUrdfString(const std::string & xml, const std::string & root_link, const std::string & tip_link);
  static Chain fromUrdfFile(const std::string & path, const std::string & root_link, const std::string & tip_link);

  const std::vector<JointInfo> & joints() const {return joints_;}
  // Actuated joints only, in chain order.
  std::vector<const JointInfo *> actuated() const;
  std::size_t dof() const {return dof_;}
  const std::string & root() const {return root_;}
  const std::string & tip() const {return tip_;}

  // Pose of the tip link in the root frame. q.size() must equal dof().
  Eigen::Isometry3d forward(const Eigen::VectorXd & q) const;
  // Geometric Jacobian (6 x dof): rows 0-2 linear velocity, rows 3-5 angular velocity, in the root frame.
  Eigen::MatrixXd jacobian(const Eigen::VectorXd & q) const;

private:
  std::vector<JointInfo> joints_;
  std::size_t dof_{0};
  std::string root_;
  std::string tip_;
};

}  // namespace yaro_check
