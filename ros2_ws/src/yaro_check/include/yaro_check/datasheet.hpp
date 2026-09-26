// Compare a URDF chain against the published data sheet values for the same robot.
#pragma once

#include "yaro_check/kinematics.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace yaro_check
{

struct DatasheetEntry
{
  std::string model;
  double payload_kg{0.0};
  double reach_mm{0.0};
  double weight_kg{0.0};
  std::array<double, 6> range_deg{};      // symmetric +/- working range per axis
  std::array<double, 6> max_speed_dps{};  // deg/s per axis
};

// Parses the whitespace table in models/datasheet.txt ('#' starts a comment).
std::vector<DatasheetEntry> parseDatasheet(const std::string & text);
std::optional<DatasheetEntry> findEntry(const std::vector<DatasheetEntry> & entries, const std::string & model);

struct Comparison
{
  std::string quantity;  // "range" | "speed"
  std::string joint;
  double urdf{0.0};       // deg or deg/s
  double datasheet{0.0};  // deg or deg/s
  bool matches{false};    // |urdf - datasheet| <= tolerance
};

// Tolerances: 0.5 deg for ranges, 0.5 deg/s for speeds (the data sheet is in whole degrees).
std::vector<Comparison> compare(const Chain & chain, const DatasheetEntry & ds);

// Largest horizontal distance of the tip from the base z axis over `samples` random
// joint configurations inside the URDF limits (fixed seed, so the number is reproducible).
double sampledHorizontalReach(const Chain & chain, std::size_t samples, unsigned seed);

}  // namespace yaro_check
