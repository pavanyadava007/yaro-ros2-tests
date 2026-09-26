#include "test_urdfs.hpp"
#include "yaro_check/limits.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

using yaro_check::Chain;
using yaro_check::Level;
using yaro_check::LimitChecker;
using yaro_check::LimitCheckerConfig;

class Limits : public ::testing::Test
{
protected:
  Chain chain = Chain::fromUrdfString(yaro_check::test::kPlanar2R, "base", "tip");
  LimitChecker checker{chain, LimitCheckerConfig{0.1, 1.0}};
};

TEST_F(Limits, InsideLimitsIsOk)
{
  const auto s = checker.check({"j1", "j2"}, {0.0, 0.5}, {0.1, -0.1});
  ASSERT_EQ(s.size(), 2u);
  EXPECT_EQ(LimitChecker::worst(s), Level::kOk);
}

TEST_F(Limits, NearLimitWarnsAndBeyondLimitErrors)
{
  auto s = checker.check({"j1", "j2"}, {2.95, 0.0}, {});
  EXPECT_EQ(s[0].level, Level::kWarn);
  EXPECT_EQ(s[1].level, Level::kOk);
  s = checker.check({"j1", "j2"}, {0.0, -2.01}, {});
  EXPECT_EQ(s[1].level, Level::kError);
  EXPECT_NE(s[1].message.find("outside"), std::string::npos);
}

TEST_F(Limits, ExactlyOnTheLimitIsNotAnError)
{
  const auto s = checker.check({"j1", "j2"}, {3.0, -2.0}, {});
  EXPECT_EQ(s[0].level, Level::kWarn);
  EXPECT_EQ(s[1].level, Level::kWarn);
}

TEST_F(Limits, VelocityAboveLimitErrorsAndScaleTightensIt)
{
  EXPECT_EQ(checker.check({"j1", "j2"}, {0.0, 0.0}, {0.0, 1.01})[1].level, Level::kError);
  EXPECT_EQ(checker.check({"j1", "j2"}, {0.0, 0.0}, {0.0, -1.01})[1].level, Level::kError);
  const LimitChecker slow(chain, LimitCheckerConfig{0.1, 0.25});
  EXPECT_EQ(slow.check({"j1", "j2"}, {0.0, 0.0}, {0.6, 0.0})[0].level, Level::kError);  // 0.6 > 0.25 * 2.0
  EXPECT_EQ(slow.check({"j1", "j2"}, {0.0, 0.0}, {0.4, 0.0})[0].level, Level::kOk);
}

TEST_F(Limits, MissingJointAndNonFiniteValuesAreErrors)
{
  auto s = checker.check({"j1"}, {0.0}, {});
  EXPECT_EQ(s[1].level, Level::kError);
  EXPECT_EQ(s[1].message, "no data");
  s = checker.check({"j2", "j1"}, {std::numeric_limits<double>::quiet_NaN(), 0.0}, {});
  EXPECT_EQ(s[0].level, Level::kOk);      // j1, order follows the chain, not the message
  EXPECT_EQ(s[1].level, Level::kError);   // j2 NaN
}

TEST_F(Limits, ExtraJointsInTheMessageAreIgnored)
{
  const auto s = checker.check({"gripper", "j2", "j1"}, {9.0, 0.0, 0.0}, {});
  ASSERT_EQ(s.size(), 2u);
  EXPECT_EQ(LimitChecker::worst(s), Level::kOk);
}

TEST_F(Limits, RejectsMalformedMessages)
{
  EXPECT_THROW(checker.check({"j1", "j2"}, {0.0}, {}), std::invalid_argument);
  EXPECT_THROW(checker.check({"j1", "j2"}, {0.0, 0.0}, {1.0}), std::invalid_argument);
  EXPECT_THROW(LimitChecker(chain, LimitCheckerConfig{-1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(LimitChecker(chain, LimitCheckerConfig{0.1, 0.0}), std::invalid_argument);
}

TEST_F(Limits, ContinuousJointHasNoPositionLimit)
{
  const auto c = Chain::fromUrdfString(yaro_check::test::kPrismaticContinuous, "base", "wheel");
  const LimitChecker k(c, LimitCheckerConfig{});
  const auto s = k.check({"slide", "spin"}, {0.2, 100.0}, {0.0, 2.9});
  EXPECT_EQ(s[1].level, Level::kOk);
  EXPECT_EQ(k.check({"slide", "spin"}, {0.2, 0.0}, {0.0, 3.1})[1].level, Level::kError);
}

TEST_F(Limits, TrajectoryVelocityRatio)
{
  Eigen::MatrixXd traj(3, 2);
  traj << 0.0, 0.0,
    0.1, 0.05,
    0.3, 0.05;
  // dt 0.1: j1 speeds 1.0 and 2.0 rad/s (limit 2.0), j2 0.5 and 0 (limit 1.0) -> worst ratio 1.0
  EXPECT_NEAR(checker.maxVelocityRatio(traj, 0.1), 1.0, 1e-12);
  EXPECT_NEAR(checker.maxVelocityRatio(traj, 0.05), 2.0, 1e-12);
  EXPECT_THROW(checker.maxVelocityRatio(traj, 0.0), std::invalid_argument);
  EXPECT_THROW(checker.maxVelocityRatio(Eigen::MatrixXd::Zero(3, 3), 0.1), std::invalid_argument);
}
