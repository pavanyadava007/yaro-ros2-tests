#include "yaro_check/model_audit.hpp"

#include <urdf/model.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace yaro_check
{

std::size_t AuditSummary::errors() const
{
  return static_cast<std::size_t>(std::count_if(findings.begin(), findings.end(), [](const Finding & f) {return f.error;}));
}

std::size_t AuditSummary::warnings() const {return findings.size() - errors();}

namespace
{

bool hasUriScheme(const std::string & f)
{
  return f.rfind("package://", 0) == 0 || f.rfind("file://", 0) == 0;
}

void checkMesh(const urdf::GeometrySharedPtr & g, const std::string & link, const char * what, AuditSummary & s)
{
  if (!g || g->type != urdf::Geometry::MESH) {
    return;
  }
  const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(g);
  if (mesh && !hasUriScheme(mesh->filename)) {
    s.findings.push_back({"mesh.uri_scheme", link, false,
        std::string(what) + " mesh '" + mesh->filename + "' has no package:// or file:// scheme"});
  }
}

}  // namespace

AuditSummary audit(const urdf::Model & model)
{
  AuditSummary s;
  s.robot = model.getName();

  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);
  s.links = links.size();
  s.joints = model.joints_.size();

  const auto roots = std::count_if(links.begin(), links.end(), [](const urdf::LinkSharedPtr & l) {return !l->parent_joint;});
  if (roots != 1) {
    s.findings.push_back({"structure.single_root", model.getName(), true, std::to_string(roots) + " root links"});
  }
  if (const auto root = model.getRoot(); root && root->inertial) {
    s.findings.push_back({"structure.root_inertia", root->name, false,
        "root link has an inertia, which KDL-based tools ignore; add a massless base link above it"});
  }

  for (const auto & [name, j] : model.joints_) {
    const bool limited = j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::PRISMATIC;
    const bool moving = limited || j->type == urdf::Joint::CONTINUOUS;
    if (moving) {
      ++s.actuated;
      const double n = std::sqrt(j->axis.x * j->axis.x + j->axis.y * j->axis.y + j->axis.z * j->axis.z);
      if (std::abs(n - 1.0) > 1e-6) {
        s.findings.push_back({"joint.axis_unit", name, true, "axis norm " + std::to_string(n)});
      }
    }
    if (limited) {
      if (!j->limits) {
        s.findings.push_back({"joint.limits_ordered", name, true, "no <limit> element"});
        continue;
      }
      if (!(j->limits->lower < j->limits->upper)) {
        s.findings.push_back({"joint.limits_ordered", name, true, "lower >= upper"});
      }
      if (!(j->limits->velocity > 0.0)) {
        s.findings.push_back({"joint.velocity_positive", name, true, "velocity limit <= 0"});
      }
      if (!(j->limits->effort > 0.0)) {
        s.findings.push_back({"joint.effort_positive", name, true, "effort limit <= 0"});
      }
    }
  }

  for (const auto & l : links) {
    checkMesh(l->visual ? l->visual->geometry : nullptr, l->name, "visual", s);
    checkMesh(l->collision ? l->collision->geometry : nullptr, l->name, "collision", s);
    if (!l->inertial) {
      continue;
    }
    const auto & in = *l->inertial;
    s.total_mass += in.mass;
    if (!(in.mass > 0.0)) {
      s.findings.push_back({"link.mass_positive", l->name, true, "mass " + std::to_string(in.mass)});
    }
    Eigen::Matrix3d inertia;
    inertia << in.ixx, in.ixy, in.ixz,
      in.ixy, in.iyy, in.iyz,
      in.ixz, in.iyz, in.izz;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(inertia);
    const Eigen::Vector3d p = es.eigenvalues();  // ascending
    const double tol = 1e-9 * std::max(1.0, p.cwiseAbs().maxCoeff());
    std::ostringstream moments;
    moments << "principal moments " << p.transpose();
    if (p.minCoeff() <= 0.0) {
      s.findings.push_back({"inertia.positive_definite", l->name, true, moments.str()});
    } else if (p[0] + p[1] < p[2] - tol) {
      s.findings.push_back({"inertia.triangle", l->name, true, moments.str() + " violate I1 + I2 >= I3"});
    }
  }
  return s;
}

AuditSummary auditString(const std::string & xml)
{
  urdf::Model model;
  if (!model.initString(xml)) {
    throw std::runtime_error("could not parse URDF");
  }
  return audit(model);
}

}  // namespace yaro_check
