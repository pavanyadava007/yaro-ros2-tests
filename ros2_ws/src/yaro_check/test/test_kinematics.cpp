#include "test_urdfs.hpp"
#include "yaro_check/kinematics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

using yaro_check::Chain;

namespace
{

const std::string kModels = YARO_MODELS_DIR;

// Central finite difference of the tip position and orientation, as an independent Jacobian reference.
Eigen::MatrixXd numericJacobian(const Chain & c, const Eigen::VectorXd & q, double h = 1e-6)
{
  Eigen::MatrixXd j(6, q.size());
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp[i] += h;
    qm[i] -= h;
    const auto tp = c.forward(qp), tm = c.forward(qm);
    j.block<3, 1>(0, i) = (tp.translation() - tm.translation()) / (2 * h);
    // dR * R^T is skew(omega) * 2h to first order
    const Eigen::Matrix3d dr = (tp.linear() - tm.linear()) * c.forward(q).linear().transpose() / (2 * h);
    j.block<3, 1>(3, i) = Eigen::Vector3d(dr(2, 1), dr(0, 2), dr(1, 0));
  }
  return j;
}

}  // namespace

TEST(Kinematics, Planar2RMatchesHandCalculation)
{
  const auto c = Chain::fromUrdfString(yaro_check::test::kPlanar2R, "base", "tip");
  ASSERT_EQ(c.dof(), 2u);
  const double q1 = 0.3, q2 = -0.7;
  const auto t = c.forward(Eigen::Vector2d(q1, q2));
  EXPECT_NEAR(t.translation().x(), std::cos(q1) + 0.5 * std::cos(q1 + q2), 1e-12);
  EXPECT_NEAR(t.translation().y(), std::sin(q1) + 0.5 * std::sin(q1 + q2), 1e-12);
  EXPECT_NEAR(t.translation().z(), 0.0, 1e-12);
  EXPECT_NEAR(std::atan2(t.linear()(1, 0), t.linear()(0, 0)), q1 + q2, 1e-12);
}

TEST(Kinematics, Planar2RJacobianMatchesHandCalculation)
{
  const auto c = Chain::fromUrdfString(yaro_check::test::kPlanar2R, "base", "tip");
  const double q1 = 0.4, q2 = 0.9;
  const auto j = c.jacobian(Eigen::Vector2d(q1, q2));
  EXPECT_NEAR(j(0, 0), -std::sin(q1) - 0.5 * std::sin(q1 + q2), 1e-12);
  EXPECT_NEAR(j(1, 0), std::cos(q1) + 0.5 * std::cos(q1 + q2), 1e-12);
  EXPECT_NEAR(j(0, 1), -0.5 * std::sin(q1 + q2), 1e-12);
  EXPECT_NEAR(j(1, 1), 0.5 * std::cos(q1 + q2), 1e-12);
  EXPECT_NEAR(j(5, 0), 1.0, 1e-12);
  EXPECT_NEAR(j(5, 1), 1.0, 1e-12);
}

TEST(Kinematics, PrismaticAndContinuousJoints)
{
  const auto c = Chain::fromUrdfString(yaro_check::test::kPrismaticContinuous, "base", "wheel");
  ASSERT_EQ(c.dof(), 2u);
  const auto t = c.forward(Eigen::Vector2d(0.25, 1.0));
  EXPECT_NEAR(t.translation().x(), 0.25, 1e-12);
  EXPECT_NEAR(t.translation().z(), 0.1, 1e-12);
  const auto j = c.jacobian(Eigen::Vector2d(0.25, 1.0));
  EXPECT_NEAR(j(0, 0), 1.0, 1e-12);            // prismatic: linear along x
  EXPECT_NEAR((j.block<3, 1>(3, 0).norm()), 0.0, 1e-12);  // no angular part
  EXPECT_TRUE(c.actuated()[1]->continuous);
}

TEST(Kinematics, RejectsBadInput)
{
  const auto c = Chain::fromUrdfString(yaro_check::test::kPlanar2R, "base", "tip");
  EXPECT_THROW(c.forward(Eigen::VectorXd::Zero(3)), std::invalid_argument);
  EXPECT_THROW(c.jacobian(Eigen::VectorXd::Zero(1)), std::invalid_argument);
  EXPECT_THROW(Chain::fromUrdfString(yaro_check::test::kPlanar2R, "base", "nope"), std::runtime_error);
  EXPECT_THROW(Chain::fromUrdfString(yaro_check::test::kPlanar2R, "tip", "base"), std::runtime_error);
  EXPECT_THROW(Chain::fromUrdfString("<robot", "a", "b"), std::runtime_error);
  EXPECT_THROW(Chain::fromUrdfFile("/does/not/exist.urdf", "a", "b"), std::runtime_error);
}

class YaroModel : public ::testing::TestWithParam<std::string> {};

TEST_P(YaroModel, SixRevoluteJointsFromBaseToFlange)
{
  const auto c = Chain::fromUrdfFile(kModels + "/" + GetParam() + "/robot.urdf", "link_0", "ee_frame");
  EXPECT_EQ(c.dof(), 6u);
  for (const auto * j : c.actuated()) {
    EXPECT_FALSE(j->prismatic) << j->name;
    EXPECT_FALSE(j->continuous) << j->name;
  }
}

TEST_P(YaroModel, AnalyticJacobianMatchesFiniteDifferences)
{
  const auto c = Chain::fromUrdfFile(kModels + "/" + GetParam() + "/robot.urdf", "link_0", "ee_frame");
  std::mt19937 rng(3);
  for (int n = 0; n < 200; ++n) {
    Eigen::VectorXd q(6);
    std::size_t i = 0;
    for (const auto * j : c.actuated()) {
      q[static_cast<Eigen::Index>(i++)] = std::uniform_real_distribution<double>(j->lower, j->upper)(rng);
    }
    const Eigen::MatrixXd diff = c.jacobian(q) - numericJacobian(c, q);
    ASSERT_LT(diff.cwiseAbs().maxCoeff(), 1e-6) << "config " << n;
  }
}

INSTANTIATE_TEST_SUITE_P(
  AllPublicModels, YaroModel,
  ::testing::Values("yaro_0808", "yaro_1105", "yaro_1115", "yaro_1310", "yaro_1608"));
