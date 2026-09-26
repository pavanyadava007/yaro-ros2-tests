// Small hand-written URDFs for tests whose expected answers can be worked out on paper.
#pragma once

#include <string>

namespace yaro_check::test
{

// Planar 2R arm in the x-y plane: link lengths 1.0 and 0.5, both joints about +z.
inline const std::string kPlanar2R = R"(
<robot name="planar2r">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" velocity="2.0" effort="10"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="1.0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="2.0" velocity="1.0" effort="10"/>
  </joint>
  <joint name="tool" type="fixed">
    <parent link="l2"/><child link="tip"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
  </joint>
</robot>)";

// One prismatic joint along +x followed by a continuous joint.
inline const std::string kPrismaticContinuous = R"(
<robot name="pc">
  <link name="base"/>
  <link name="slider"/>
  <link name="wheel"/>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="slider"/>
    <axis xyz="1 0 0"/>
    <limit lower="0.0" upper="0.5" velocity="0.2" effort="100"/>
  </joint>
  <joint name="spin" type="continuous">
    <parent link="slider"/><child link="wheel"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit velocity="3.0" effort="5"/>
  </joint>
</robot>)";

}  // namespace yaro_check::test
