#include "jrl/Dataset.h"

#include <sstream>

namespace jrl {
/**********************************************************************************************************************/
Dataset::Dataset(const std::string& name, std::vector<char>& robots, std::map<char, std::vector<Entry>> measurements,
        boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>>& ground_truth,
        boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>>& initial_estimates)
    : name_(name),
      robots_(robots),
      measurements_(measurements),
      ground_truth_(ground_truth),
      initial_estimates_(initial_estimates) {}

/**********************************************************************************************************************/
std::string Dataset::name() { return name_; }

/**********************************************************************************************************************/
std::vector<char> Dataset::robots() { return robots_; }

/**********************************************************************************************************************/
std::pair<gtsam::Values, ValueTypes> Dataset::groundTruthWithTypes(const boost::optional<char>& robot_id) {
  return accessor<std::pair<gtsam::Values, ValueTypes>>("groundTruth", ground_truth_, robot_id);
}

gtsam::Values Dataset::groundTruth(const boost::optional<char>& robot_id) {
  return groundTruthWithTypes(robot_id).first;
}

/**********************************************************************************************************************/
std::pair<gtsam::Values, ValueTypes> Dataset::initializationWithTypes(const boost::optional<char>& robot_id) {
  return accessor<std::pair<gtsam::Values, ValueTypes>>("initialization", initial_estimates_, robot_id);
}
gtsam::Values Dataset::initialization(const boost::optional<char>& robot_id) {
  return initializationWithTypes(robot_id).first;
}

/**********************************************************************************************************************/
std::vector<Entry> Dataset::measurements(const boost::optional<char>& robot_id) {
  return accessor<std::vector<Entry>>("measurements", measurements_, robot_id);
}

/**********************************************************************************************************************/
template <typename RETURN_TYPE>
RETURN_TYPE Dataset::accessor(const std::string& func_name, boost::optional<std::map<char, RETURN_TYPE>> robot_mapping,
                              const boost::optional<char>& robot_id) {
  if (robot_mapping == boost::none) {
    std::stringstream stream;
    stream << "Dataset:" << func_name << "requested but dataset does not contain " << func_name << "information";
    throw std::runtime_error(stream.str());
  } else if (robot_id == boost::none) {
    if (num_robots_ == 0) {
      return (*robot_mapping)[robots_[0]];
    } else {
      std::stringstream stream;
      stream << "Dataset:" << func_name;
      stream << " called without a robot_id, but Dataset is a multi-robot dataset.";
      stream << "Please request data for one of the following robots: ";
      for (auto& rid : robots_) {
        stream << rid << " ";
      }
      throw std::runtime_error(stream.str());
    }
  } else {
    try {
      return (*robot_mapping)[*robot_id];
    } catch (std::out_of_range& oor) {
      std::stringstream stream;
      stream << "Dataset:" << func_name;
      stream << " called with invalid robot_id (" << *robot_id << ").";
      stream << "This dataset contains data for the following robots: ";
      for (auto& rid : robots_) {
        stream << rid << " ";
      }
      throw std::out_of_range(stream.str());
    }
  }
}

}  // namespace jrl