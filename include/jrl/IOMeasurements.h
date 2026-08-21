/// @brief This module implements all functionality for input and output of measurements.
#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <nlohmann/json.hpp>
#include <stdexcept>
using json = nlohmann::json;
namespace jrl {

// Define statically the tags for measurements
static const std::string BetweenFactorPose2Tag = "BetweenFactorPose2";
static const std::string BetweenFactorPose3Tag = "BetweenFactorPose3";
static const std::string PriorFactorPose2Tag = "PriorFactorPose2";
static const std::string PriorFactorPose3Tag = "PriorFactorPose3";
static const std::string RangeFactorPose2Tag = "RangeFactorPose2";
static const std::string RangeFactorPose3Tag = "RangeFactorPose3";
static const std::string RangeFactor2DTag = "RangeFactor2D";
static const std::string RangeFactor3DTag = "RangeFactor3D";
static const std::string RangeFactorWithTransformPose2Tag = "RangeFactorWithTransformPose2";
static const std::string RangeFactorWithTransformPose3Tag = "RangeFactorWithTransformPose3";
static const std::string RangeFactorWithTransform2DTag = "RangeFactorWithTransform2D";
static const std::string RangeFactorWithTransform3DTag = "RangeFactorWithTransform3D";
static const std::string BearingRangeFactorPose2Tag = "BearingRangeFactorPose2";
static const std::string BearingRangeFactorPose3Tag = "BearingRangeFactorPose3";
static const std::string BearingRangeFactor2DTag = "BearingRangeFactor2D";
static const std::string BearingRangeFactor3DTag = "BearingRangeFactor3D";
static const std::string BetweenFactorPoint2Tag = "BetweenFactorPoint2";
static const std::string BetweenFactorPoint3Tag = "BetweenFactorPoint3";
static const std::string PriorFactorPoint2Tag = "PriorFactorPoint2";
static const std::string PriorFactorPoint3Tag = "PriorFactorPoint3";
static const std::string PriorFactorConstantBiasTag = "PriorFactorConstantBias";
static const std::string CombinedImuFactorTag = "CombinedImuFactor";

// GTSAM does not expose RangeFactorWithTransform's body_T_sensor. Retain the
// parsed value so JRL can serialize factors it constructed without losing it.
template <typename A1, typename A2 = A1>
class RangeFactorWithTransform final : public gtsam::RangeFactorWithTransform<A1, A2> {
 public:
  using Base = gtsam::RangeFactorWithTransform<A1, A2>;

  RangeFactorWithTransform(gtsam::Key key1, gtsam::Key key2, double measured, const gtsam::SharedNoiseModel& model,
                           const A1& body_T_sensor)
      : Base(key1, key2, measured, model, body_T_sensor), body_T_sensor_(body_T_sensor) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new RangeFactorWithTransform(*this));
  }

  const A1& body_T_sensor() const { return body_T_sensor_; }

 private:
  A1 body_T_sensor_;
};

namespace io_measurements {

/**********************************************************************************************************************/
/// @brief Parses a covariance matrix from json
gtsam::Matrix parseCovariance(json input_json, int d);
json serializeCovariance(gtsam::Matrix covariance);

/**********************************************************************************************************************/
gtsam::NonlinearFactor::shared_ptr parseCombinedImuFactor(const json& input_json);
json serializeCombinedImuFactor(gtsam::NonlinearFactor::shared_ptr& factor);

/**********************************************************************************************************************/
template <typename T>
gtsam::NonlinearFactor::shared_ptr parsePrior(std::function<T(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key_json = input_json["key"];
  json measurement_json = input_json["prior"];
  json covariance_json = input_json["covariance"];

  // Construct the factor
  T measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<T>::GetDimension(measured);
  typename gtsam::Matrix covariance = parseCovariance(input_json["covariance"], d);
  typename gtsam::PriorFactor<T>::shared_ptr factor = boost::make_shared<gtsam::PriorFactor<T>>(
      key_json.get<uint64_t>(), measured, gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename T>
json serializePrior(std::function<json(T)> val_serializer_fn, std::string type_tag,
                    gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename gtsam::PriorFactor<T>::shared_ptr prior = boost::dynamic_pointer_cast<gtsam::PriorFactor<T>>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(prior->noiseModel());

  output["type"] = type_tag;
  output["key"] = prior->key();
  output["prior"] = val_serializer_fn(prior->prior());

  output["covariance"] = serializeCovariance(noise_model->covariance());
  return output;
}

template <typename A1, typename A2 = A1>
gtsam::NonlinearFactor::shared_ptr parseRangeFactorWithTransform(std::function<double(json)> measurement_parser_fn,
                                                                 std::function<A1(json)> transform_parser_fn,
                                                                 json input_json) {
  auto key1 = input_json["key1"].get<uint64_t>();
  auto key2 = input_json["key2"].get<uint64_t>();
  double measured = measurement_parser_fn(input_json["measurement"]);
  A1 body_T_sensor = transform_parser_fn(input_json["body_T_sensor"]);
  Eigen::MatrixXd covariance = parseCovariance(input_json["covariance"], 1);

  typename RangeFactorWithTransform<A1, A2>::shared_ptr factor = boost::make_shared<RangeFactorWithTransform<A1, A2>>(
      key1, key2, measured, gtsam::noiseModel::Gaussian::Covariance(covariance), body_T_sensor);
  return factor;
}

template <typename A1, typename A2 = A1>
json serializeRangeFactorWithTransform(std::function<json(double)> measurement_serializer_fn,
                                       std::function<json(A1)> transform_serializer_fn, std::string type_tag,
                                       gtsam::NonlinearFactor::shared_ptr& factor) {
  const auto range = boost::dynamic_pointer_cast<RangeFactorWithTransform<A1, A2>>(factor);
  if (!range) {
    throw std::invalid_argument("RangeFactorWithTransform must be created by JRL to serialize body_T_sensor");
  }
  const auto noise_model = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(range->noiseModel());

  json output;
  output["type"] = type_tag;
  output["key1"] = range->keys().front();
  output["key2"] = range->keys().back();
  output["measurement"] = measurement_serializer_fn(range->measured());
  output["covariance"] = serializeCovariance(noise_model->covariance());
  output["body_T_sensor"] = transform_serializer_fn(range->body_T_sensor());
  return output;
}

/**********************************************************************************************************************/
template <typename MEASURE, typename FACTOR>
gtsam::NonlinearFactor::shared_ptr parseNoiseModel2(std::function<MEASURE(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key1_json = input_json["key1"];
  json key2_json = input_json["key2"];
  json measurement_json = input_json["measurement"];
  json covariance_json = input_json["covariance"];

  // Construct the factor
  MEASURE measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<MEASURE>::GetDimension(measured);
  typename FACTOR::shared_ptr factor =
      boost::make_shared<FACTOR>(key1_json.get<uint64_t>(), key2_json.get<uint64_t>(), measured,
                                 gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename MEASURE, typename FACTOR>
json serializeNoiseModel2(std::function<json(MEASURE)> val_serializer_fn, std::string type_tag,
                          gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename FACTOR::shared_ptr between = boost::dynamic_pointer_cast<FACTOR>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(between->noiseModel());
  output["type"] = type_tag;
  output["key1"] = between->keys().front();
  output["key2"] = between->keys().back();
  output["measurement"] = val_serializer_fn(between->measured());
  output["covariance"] = serializeCovariance(noise_model->covariance());
  return output;
}

}  // namespace io_measurements

}  // namespace jrl
