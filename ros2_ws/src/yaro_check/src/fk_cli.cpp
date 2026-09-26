// yaro_fk MODEL_DIR q1 q2 q3 q4 q5 q6: prints the ee_frame position (m) in the link_0 frame as JSON.
#include "yaro_check/kinematics.hpp"

#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 8) {
    std::cerr << "usage: yaro_fk MODEL_DIR q1 q2 q3 q4 q5 q6\n";
    return 2;
  }
  try {
    const auto chain = yaro_check::Chain::fromUrdfFile(std::string(argv[1]) + "/robot.urdf", "link_0", "ee_frame");
    Eigen::VectorXd q(6);
    for (int i = 0; i < 6; ++i) {
      q[i] = std::stod(argv[i + 2]);
    }
    const Eigen::Vector3d p = chain.forward(q).translation();
    std::cout << std::setprecision(12) << "[" << p.x() << ", " << p.y() << ", " << p.z() << "]\n";
  } catch (const std::exception & e) {
    std::cerr << "yaro_fk: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
