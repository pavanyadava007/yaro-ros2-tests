#include "yaro_check/datasheet.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>

namespace yaro_check
{

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;
}

std::vector<DatasheetEntry> parseDatasheet(const std::string & text)
{
  std::vector<DatasheetEntry> out;
  std::istringstream lines(text);
  std::string line;
  int lineno = 0;
  while (std::getline(lines, line)) {
    ++lineno;
    line = line.substr(0, line.find('#'));
    std::istringstream in(line);
    DatasheetEntry e;
    if (!(in >> e.model)) {
      continue;  // blank or comment
    }
    in >> e.payload_kg >> e.reach_mm >> e.weight_kg;
    for (auto & r : e.range_deg) {
      in >> r;
    }
    for (auto & s : e.max_speed_dps) {
      in >> s;
    }
    if (in.fail()) {
      throw std::runtime_error("datasheet line " + std::to_string(lineno) + ": expected 16 values after the model name");
    }
    std::string extra;
    if (in >> extra) {
      throw std::runtime_error("datasheet line " + std::to_string(lineno) + ": too many values");
    }
    out.push_back(e);
  }
  return out;
}

std::optional<DatasheetEntry> findEntry(const std::vector<DatasheetEntry> & entries, const std::string & model)
{
  const auto it = std::find_if(entries.begin(), entries.end(), [&](const DatasheetEntry & e) {return e.model == model;});
  if (it == entries.end()) {
    return std::nullopt;
  }
  return *it;
}

std::vector<Comparison> compare(const Chain & chain, const DatasheetEntry & ds)
{
  const auto joints = chain.actuated();
  if (joints.size() != ds.range_deg.size()) {
    throw std::invalid_argument("chain has " + std::to_string(joints.size()) + " actuated joints, data sheet has 6 axes");
  }
  std::vector<Comparison> out;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto & j = *joints[i];
    // Symmetric range: compare the half-range; an asymmetric URDF range shows up as a mismatch.
    const double half = 0.5 * (j.upper - j.lower) * kRadToDeg;
    const double centre = 0.5 * (j.upper + j.lower) * kRadToDeg;
    Comparison r{"range", j.name, half, ds.range_deg[i], false};
    r.matches = std::abs(half - ds.range_deg[i]) <= 0.5 && std::abs(centre) <= 0.5;
    out.push_back(r);
    Comparison s{"speed", j.name, j.velocity * kRadToDeg, ds.max_speed_dps[i], false};
    s.matches = std::abs(s.urdf - s.datasheet) <= 0.5;
    out.push_back(s);
  }
  return out;
}

double sampledHorizontalReach(const Chain & chain, std::size_t samples, unsigned seed)
{
  std::mt19937 rng(seed);
  const auto joints = chain.actuated();
  Eigen::VectorXd q(static_cast<Eigen::Index>(joints.size()));
  double best = 0.0;
  for (std::size_t n = 0; n < samples; ++n) {
    for (std::size_t i = 0; i < joints.size(); ++i) {
      std::uniform_real_distribution<double> d(joints[i]->lower, joints[i]->upper);
      q[static_cast<Eigen::Index>(i)] = d(rng);
    }
    const Eigen::Vector3d p = chain.forward(q).translation();
    best = std::max(best, std::hypot(p.x(), p.y()));
  }
  return best;
}

}  // namespace yaro_check
