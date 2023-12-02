#pragma once

#include <vector>
#include <ostream>

namespace wspltr
{
struct DHParameters {
  std::vector<double> offset {};
  std::vector<double> a_length {};
  std::vector<double> d_length {};
  std::vector<double> alpha {};
  std::vector<double> min_limit {};
  std::vector<double> max_limit {};

  friend std::ostream& operator<<(std::ostream& os, const DHParameters& parameters)
  {
    os << "DH Parameters: " << std::endl;

    os << " Offsets: ";
    for (const auto& offset: parameters.offset) {
      os << offset << " ";
    }
    os << std::endl;

    os << " Link Lengths: ";
    for (const auto& link_length: parameters.a_length) {
      os << link_length << " ";
    }
    os << std::endl;

    os << " Link Displacements: ";
    for (const auto& link_displacement: parameters.d_length) {
      os << link_displacement << " ";
    }
    os << std::endl;

    os << " Link Twists: ";
    for (const auto& link_twist: parameters.alpha) {
      os << link_twist << " ";
    }
    os << std::endl;

    os << " Minimum Limits: ";
    for (const auto& minimum_limit: parameters.min_limit) {
      os << minimum_limit << " ";
    }
    os << std::endl;

    os << " Maximum Limits: ";
    for (const auto& maximum_limit: parameters.max_limit) {
      os << maximum_limit << " ";
    }
    os << std::endl;

    return os;
  }

};
}
