// Independent reference: KDL (orocos) builds its own chain from the same URDF.
// Our FK and Jacobian must agree with it to numerical precision on every public YARO model.
#include "yaro_check/kinematics.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <gtest/gtest.h>

#include <random>
#include <sstream>
#include <string>

namespace
{
const std::string kModels = YARO_MODELS_DIR;
}

class KdlCrossCheck : public ::testing::TestWithParam<std::string> {};

TEST_P(KdlCrossCheck, ForwardKinematicsAndJacobianAgree)
{
  urdf::Model model;
  ASSERT_TRUE(model.initFile(kModels + "/" + GetParam() + "/robot.urdf"));
  const auto chain = yaro_check::Chain::fromUrdf(model, "link_0", "ee_frame");

  KDL::Tree tree;
  ASSERT_TRUE(kdl_parser::treeFromUrdfModel(model, tree));
  KDL::Chain kchain;
  ASSERT_TRUE(tree.getChain("link_0", "ee_frame", kchain));
  ASSERT_EQ(kchain.getNrOfJoints(), chain.dof());
  KDL::ChainFkSolverPos_recursive fk(kchain);
  KDL::ChainJntToJacSolver jac(kchain);

  std::mt19937 rng(42);
  double worst_pos = 0.0, worst_rot = 0.0, worst_jac = 0.0;
  for (int n = 0; n < 1000; ++n) {
    Eigen::VectorXd q(6);
    KDL::JntArray kq(6);
    std::size_t i = 0;
    for (const auto * j : chain.actuated()) {
      const double v = std::uniform_real_distribution<double>(j->lower, j->upper)(rng);
      q[static_cast<Eigen::Index>(i)] = v;
      kq(static_cast<unsigned>(i)) = v;
      ++i;
    }
    KDL::Frame f;
    ASSERT_GE(fk.JntToCart(kq, f), 0);
    const auto t = chain.forward(q);
    for (int r = 0; r < 3; ++r) {
      worst_pos = std::max(worst_pos, std::abs(t.translation()[r] - f.p(r)));
      for (int c = 0; c < 3; ++c) {
        worst_rot = std::max(worst_rot, std::abs(t.linear()(r, c) - f.M(r, c)));
      }
    }
    KDL::Jacobian kj(6);
    ASSERT_GE(jac.JntToJac(kq, kj), 0);
    const auto j = chain.jacobian(q);
    for (unsigned r = 0; r < 6; ++r) {
      for (unsigned c = 0; c < 6; ++c) {
        worst_jac = std::max(worst_jac, std::abs(j(r, c) - kj(r, c)));
      }
    }
  }
  const auto sci = [](double v) {std::ostringstream o; o << std::scientific << v; return o.str();};
  RecordProperty("worst_position_error_m", sci(worst_pos));
  RecordProperty("worst_rotation_error", sci(worst_rot));
  RecordProperty("worst_jacobian_error", sci(worst_jac));
  EXPECT_LT(worst_pos, 1e-12);
  EXPECT_LT(worst_rot, 1e-12);
  EXPECT_LT(worst_jac, 1e-12);
}

INSTANTIATE_TEST_SUITE_P(
  AllPublicModels, KdlCrossCheck,
  ::testing::Values("yaro_0808", "yaro_1105", "yaro_1115", "yaro_1310", "yaro_1608"));
