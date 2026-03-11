#pragma once

#include <string>

namespace turbolib::structure {
struct ShooterSetpoint {
  std::string name;
  double rpm;
  double angle;
};
}  // namespace turbolib::structure
