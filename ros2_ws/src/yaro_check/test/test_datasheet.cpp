#include "test_urdfs.hpp"
#include "yaro_check/datasheet.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using namespace yaro_check;

namespace
{

// 6R arm with known limits: +/- 180 deg and 90 deg/s on every joint, except joint 3 (+/- 90 deg, 180 deg/s).
std::string sixR()
{
  std::string x = "<robot name='six'><link name='l0'/>";
  for (int i = 1; i <= 6; ++i) {
    const double range = i == 3 ? M_PI / 2 : M_PI;
    const double vel = i == 3 ? M_PI : M_PI / 2;
    x += "<link name='l" + std::to_string(i) + "'/><joint name='j" + std::to_string(i) +
      "' type='revolute'><parent link='l" + std::to_string(i - 1) + "'/><child link='l" + std::to_string(i) +
      "'/><origin xyz='0 0 0.1'/><axis xyz='0 0 1'/><limit lower='" + std::to_string(-range) + "' upper='" +
      std::to_string(range) + "' velocity='" + std::to_string(vel) + "' effort='1'/></joint>";
  }
  return x + "</robot>";
}

}  // namespace

TEST(Datasheet, ParsesTableAndComments)
{
  const auto e = parseDatasheet(
    "# header\n\nm1 5 1100 28  360 360 165 360 360 360  228 228 270 420 420 420  # trailing\n");
  ASSERT_EQ(e.size(), 1u);
  EXPECT_EQ(e[0].model, "m1");
  EXPECT_DOUBLE_EQ(e[0].reach_mm, 1100);
  EXPECT_DOUBLE_EQ(e[0].range_deg[2], 165);
  EXPECT_DOUBLE_EQ(e[0].max_speed_dps[5], 420);
  EXPECT_TRUE(findEntry(e, "m1").has_value());
  EXPECT_FALSE(findEntry(e, "m2").has_value());
}

TEST(Datasheet, RejectsShortAndLongLines)
{
  EXPECT_THROW(parseDatasheet("m1 5 1100 28 360\n"), std::runtime_error);
  EXPECT_THROW(parseDatasheet("m1 5 1100 28 1 2 3 4 5 6 1 2 3 4 5 6 7\n"), std::runtime_error);
}

TEST(Datasheet, CompareFlagsOnlyTheDifferingValues)
{
  const auto c = Chain::fromUrdfString(sixR(), "l0", "l6");
  DatasheetEntry ds;
  ds.range_deg = {180, 180, 90, 180, 180, 170};           // j6 differs
  ds.max_speed_dps = {90, 90, 180, 90, 90, 90};
  const auto cmp = compare(c, ds);
  ASSERT_EQ(cmp.size(), 12u);
  for (const auto & x : cmp) {
    const bool expect_match = !(x.quantity == "range" && x.joint == "j6");
    EXPECT_EQ(x.matches, expect_match) << x.quantity << " " << x.joint;
  }
}

TEST(Datasheet, CompareNeedsSixAxes)
{
  const auto c = Chain::fromUrdfString(yaro_check::test::kPlanar2R, "base", "tip");
  EXPECT_THROW(compare(c, DatasheetEntry{}), std::invalid_argument);
}

TEST(Datasheet, SampledReachOfPlanarArm)
{
  const auto c = Chain::fromUrdfString(yaro_check::test::kPlanar2R, "base", "tip");
  const double r = sampledHorizontalReach(c, 20000, 1);
  EXPECT_LE(r, 1.5 + 1e-12);   // can never exceed the sum of link lengths
  EXPECT_GT(r, 1.49);          // and gets close with 20k samples
  EXPECT_DOUBLE_EQ(r, sampledHorizontalReach(c, 20000, 1));  // reproducible
}
