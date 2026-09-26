// yaro_bench MODEL_DIR: times forward kinematics and the Jacobian of yaro_check::Chain
// against KDL on the same random configurations. Prints JSON.
#include "yaro_check/kinematics.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <vector>

namespace
{

template<typename F>
double medianNsPerCall(F && f, std::size_t calls, int repeats)
{
  std::vector<double> runs;
  for (int r = 0; r < repeats; ++r) {
    const auto t0 = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < calls; ++i) {
      f(i);
    }
    const auto t1 = std::chrono::steady_clock::now();
    runs.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() / static_cast<double>(calls));
  }
  std::sort(runs.begin(), runs.end());
  return runs[runs.size() / 2];
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: yaro_bench MODEL_DIR\n";
    return 2;
  }
  std::ifstream in(std::string(argv[1]) + "/robot.urdf");
  std::stringstream ss;
  ss << in.rdbuf();
  urdf::Model model;
  if (!model.initString(ss.str())) {
    std::cerr << "cannot parse URDF\n";
    return 1;
  }
  const auto chain = yaro_check::Chain::fromUrdf(model, "link_0", "ee_frame");
  KDL::Tree tree;
  KDL::Chain kchain;
  if (!kdl_parser::treeFromUrdfModel(model, tree) || !tree.getChain("link_0", "ee_frame", kchain)) {
    std::cerr << "KDL chain failed\n";
    return 1;
  }
  KDL::ChainFkSolverPos_recursive kfk(kchain);
  KDL::ChainJntToJacSolver kjac(kchain);

  constexpr std::size_t kConfigs = 4096;
  constexpr std::size_t kCalls = 400000;
  constexpr int kRepeats = 7;
  std::mt19937 rng(11);
  std::vector<Eigen::VectorXd> qs;
  std::vector<KDL::JntArray> kqs;
  for (std::size_t n = 0; n < kConfigs; ++n) {
    Eigen::VectorXd q(6);
    KDL::JntArray kq(6);
    std::size_t i = 0;
    for (const auto * j : chain.actuated()) {
      q[static_cast<Eigen::Index>(i)] = std::uniform_real_distribution<double>(j->lower, j->upper)(rng);
      kq(static_cast<unsigned>(i)) = q[static_cast<Eigen::Index>(i)];
      ++i;
    }
    qs.push_back(q);
    kqs.push_back(kq);
  }

  volatile double sink = 0.0;
  KDL::Frame kf;
  KDL::Jacobian kj(6);
  const double fk = medianNsPerCall([&](std::size_t i) {sink = sink + chain.forward(qs[i % kConfigs]).translation().x();}, kCalls, kRepeats);
  const double fk_kdl = medianNsPerCall([&](std::size_t i) {kfk.JntToCart(kqs[i % kConfigs], kf); sink = sink + kf.p.x();}, kCalls, kRepeats);
  const double jac = medianNsPerCall([&](std::size_t i) {sink = sink + chain.jacobian(qs[i % kConfigs])(0, 0);}, kCalls, kRepeats);
  const double jac_kdl = medianNsPerCall([&](std::size_t i) {kjac.JntToJac(kqs[i % kConfigs], kj); sink = sink + kj(0, 0);}, kCalls, kRepeats);

  std::cout << "{\"model\": \"" << argv[1] << "\", \"calls\": " << kCalls << ", \"repeats\": " << kRepeats
            << ", \"fk_ns\": " << fk << ", \"fk_kdl_ns\": " << fk_kdl << ", \"jacobian_ns\": " << jac
            << ", \"jacobian_kdl_ns\": " << jac_kdl << "}\n";
  return 0;
}
