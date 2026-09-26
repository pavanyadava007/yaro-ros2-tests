// yaro_audit --datasheet models/datasheet.txt models/yaro_1105 [models/yaro_0808 ...]
// Prints one JSON document with the model audit, the data sheet comparison and the sampled reach.
#include "yaro_check/datasheet.hpp"
#include "yaro_check/kinematics.hpp"
#include "yaro_check/model_audit.hpp"

#include <urdf/model.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string esc(const std::string & s)
{
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') {
      o += '\\';
    }
    o += c;
  }
  return o;
}

constexpr std::size_t kReachSamples = 200000;
constexpr unsigned kSeed = 7;

}  // namespace

int main(int argc, char ** argv)
{
  std::string datasheet_path;
  std::vector<std::string> dirs;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--datasheet" && i + 1 < argc) {
      datasheet_path = argv[++i];
    } else {
      dirs.push_back(a);
    }
  }
  if (datasheet_path.empty() || dirs.empty()) {
    std::cerr << "usage: yaro_audit --datasheet FILE MODEL_DIR...\n";
    return 2;
  }

  try {
    const auto entries = yaro_check::parseDatasheet(readFile(datasheet_path));
    std::cout << std::setprecision(6) << "{\n  \"reach_samples\": " << kReachSamples << ",\n  \"seed\": " << kSeed
              << ",\n  \"models\": [\n";
    for (std::size_t m = 0; m < dirs.size(); ++m) {
      const std::string name = std::filesystem::path(dirs[m]).filename().string();
      const std::string xml = readFile(dirs[m] + "/robot.urdf");
      urdf::Model model;
      if (!model.initString(xml)) {
        throw std::runtime_error("could not parse " + dirs[m]);
      }
      const auto a = yaro_check::audit(model);
      const auto chain = yaro_check::Chain::fromUrdf(model, "link_0", "ee_frame");

      std::cout << "    {\n      \"model\": \"" << esc(name) << "\",\n      \"robot_name\": \"" << esc(a.robot)
                << "\",\n      \"links\": " << a.links << ", \"joints\": " << a.joints << ", \"actuated\": " << a.actuated
                << ",\n      \"total_mass_kg\": " << a.total_mass << ",\n      \"errors\": " << a.errors()
                << ", \"warnings\": " << a.warnings() << ",\n      \"findings\": [";
      for (std::size_t f = 0; f < a.findings.size(); ++f) {
        const auto & x = a.findings[f];
        std::cout << (f ? "," : "") << "\n        {\"rule\": \"" << x.rule << "\", \"element\": \"" << esc(x.element)
                  << "\", \"error\": " << (x.error ? "true" : "false") << ", \"detail\": \"" << esc(x.detail) << "\"}";
      }
      std::cout << (a.findings.empty() ? "" : "\n      ") << "],\n";

      const double reach = yaro_check::sampledHorizontalReach(chain, kReachSamples, kSeed);
      std::cout << "      \"sampled_horizontal_reach_mm\": " << reach * 1000.0 << ",\n";
      const auto ds = yaro_check::findEntry(entries, name);
      if (!ds) {
        std::cout << "      \"datasheet\": null\n";
      } else {
        std::cout << "      \"datasheet\": {\"payload_kg\": " << ds->payload_kg << ", \"reach_mm\": " << ds->reach_mm
                  << ", \"weight_kg\": " << ds->weight_kg << "},\n      \"comparisons\": [";
        const auto cmp = yaro_check::compare(chain, *ds);
        for (std::size_t c = 0; c < cmp.size(); ++c) {
          const auto & x = cmp[c];
          std::cout << (c ? "," : "") << "\n        {\"quantity\": \"" << x.quantity << "\", \"joint\": \"" << x.joint
                    << "\", \"urdf\": " << x.urdf << ", \"datasheet\": " << x.datasheet
                    << ", \"matches\": " << (x.matches ? "true" : "false") << "}";
        }
        std::cout << "\n      ]\n";
      }
      std::cout << "    }" << (m + 1 < dirs.size() ? "," : "") << "\n";
    }
    std::cout << "  ]\n}\n";
  } catch (const std::exception & e) {
    std::cerr << "yaro_audit: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
