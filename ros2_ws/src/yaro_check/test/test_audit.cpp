#include "yaro_check/model_audit.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

using yaro_check::auditString;

namespace
{

bool has(const yaro_check::AuditSummary & s, const std::string & rule, const std::string & element)
{
  return std::any_of(s.findings.begin(), s.findings.end(), [&](const yaro_check::Finding & f) {
             return f.rule == rule && f.element == element;
           });
}

std::string oneLink(const std::string & inertial, const std::string & extra = "")
{
  return "<robot name='r'><link name='a'>" + inertial + "</link>" + extra + "</robot>";
}

std::string inertial(double m, double ixx, double iyy, double izz)
{
  return "<inertial><mass value='" + std::to_string(m) + "'/><inertia ixx='" + std::to_string(ixx) +
         "' ixy='0' ixz='0' iyy='" + std::to_string(iyy) + "' iyz='0' izz='" + std::to_string(izz) + "'/></inertial>";
}

}  // namespace

TEST(Audit, CleanModelHasNoFindings)
{
  const auto s = auditString(
    "<robot name='r'><link name='base'/><link name='a'>" + inertial(1.0, 0.1, 0.1, 0.1) +
    "</link><joint name='f' type='fixed'><parent link='base'/><child link='a'/></joint></robot>");
  EXPECT_EQ(s.errors(), 0u);
  EXPECT_EQ(s.warnings(), 0u);
  EXPECT_DOUBLE_EQ(s.total_mass, 1.0);
}

TEST(Audit, RootLinkWithInertiaIsAWarning)
{
  const auto s = auditString(oneLink(inertial(1.0, 0.1, 0.1, 0.1)));
  EXPECT_TRUE(has(s, "structure.root_inertia", "a"));
  EXPECT_EQ(s.errors(), 0u);
}

// Regression test on the published descriptions (pinned commits in models/PINNED_COMMITS.txt):
// physically valid (no errors), 14 mesh paths without a URI scheme and an inertial root link each.
class PublicModel : public ::testing::TestWithParam<std::string> {};

TEST_P(PublicModel, KnownAuditResult)
{
  std::ifstream in(std::string(YARO_MODELS_DIR) + "/" + GetParam() + "/robot.urdf");
  std::stringstream ss;
  ss << in.rdbuf();
  const auto s = auditString(ss.str());
  EXPECT_EQ(s.errors(), 0u);
  EXPECT_EQ(s.links, 8u);
  EXPECT_EQ(s.actuated, 6u);
  const auto count = [&](const std::string & rule) {
      return std::count_if(s.findings.begin(), s.findings.end(), [&](const yaro_check::Finding & f) {return f.rule == rule;});
    };
  EXPECT_EQ(count("mesh.uri_scheme"), 14);
  EXPECT_EQ(count("structure.root_inertia"), 1);
  EXPECT_EQ(s.warnings(), 15u);
}

INSTANTIATE_TEST_SUITE_P(
  AllPublicModels, PublicModel,
  ::testing::Values("yaro_0808", "yaro_1105", "yaro_1115", "yaro_1310", "yaro_1608"));

TEST(Audit, InertiaRules)
{
  EXPECT_TRUE(has(auditString(oneLink(inertial(1.0, 0.1, 0.1, 0.0))), "inertia.positive_definite", "a"));
  EXPECT_TRUE(has(auditString(oneLink(inertial(1.0, 0.1, 0.1, 0.3))), "inertia.triangle", "a"));  // 0.1 + 0.1 < 0.3
  EXPECT_FALSE(has(auditString(oneLink(inertial(1.0, 0.1, 0.1, 0.2))), "inertia.triangle", "a"));  // flat plate, boundary
}

TEST(Audit, MassRule)
{
  EXPECT_TRUE(has(auditString(oneLink(inertial(0.0, 0.1, 0.1, 0.1))), "link.mass_positive", "a"));
}

TEST(Audit, JointRules)
{
  const std::string joint =
    "<link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/>"
    "<axis xyz='0 0 2'/><limit lower='1' upper='-1' velocity='0' effort='0'/></joint>";
  const auto s = auditString(oneLink("", joint));
  EXPECT_TRUE(has(s, "joint.limits_ordered", "j"));
  EXPECT_TRUE(has(s, "joint.velocity_positive", "j"));
  EXPECT_TRUE(has(s, "joint.effort_positive", "j"));
  EXPECT_EQ(s.actuated, 1u);
}

TEST(Audit, MeshWithoutUriSchemeIsAWarning)
{
  const auto s = auditString(oneLink("<visual><geometry><mesh filename='../meshes/a.stl'/></geometry></visual>"));
  ASSERT_TRUE(has(s, "mesh.uri_scheme", "a"));
  EXPECT_EQ(s.errors(), 0u);
  EXPECT_FALSE(has(auditString(oneLink("<visual><geometry><mesh filename='package://p/a.stl'/></geometry></visual>")),
    "mesh.uri_scheme", "a"));
}
