#include <gtsam/inference/Symbol.h>
#include <gtsam/sam/RangeFactor.h>
#include <jrl/IOMeasurements.h>
#include <jrl/IOValues.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>

#include "gtest/gtest.h"

namespace {

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;
using jrl::io_values::serialize;

json measurement(const std::string& tag, gtsam::Key key1, gtsam::Key key2, double range, json body_T_sensor) {
  return {{"type", tag},          {"key1", key1},
          {"key2", key2},         {"measurement", serialize<double>(range)},
          {"covariance", {0.01}}, {"body_T_sensor", body_T_sensor}};
}

gtsam::NonlinearFactor::shared_ptr parse(const json& factor) {
  jrl::Parser parser;
  return parser.parseMeasurements({{{"stamp", 0}, {"measurements", {factor}}}})[0].measurements.at(0);
}

gtsam::NonlinearFactor::shared_ptr roundTrip(const json& factor) {
  const auto parsed = parse(factor);
  gtsam::NonlinearFactorGraph graph;
  graph.push_back(parsed);
  const jrl::Entry entry(0, {factor.at("type").get<std::string>()}, graph);
  const jrl::Writer writer;
  const json serialized = writer.serializeMeasurements({entry});
  const json& restored = serialized.at(0).at("measurements").at(0);
  EXPECT_EQ(factor.at("type"), restored.at("type"));
  EXPECT_EQ(factor.at("key1"), restored.at("key1"));
  EXPECT_EQ(factor.at("key2"), restored.at("key2"));
  EXPECT_EQ(factor.at("measurement"), restored.at("measurement"));
  EXPECT_EQ(factor.at("body_T_sensor"), restored.at("body_T_sensor"));
  EXPECT_NEAR(factor.at("covariance").at(0).get<double>(), restored.at("covariance").at(0).get<double>(), 1e-12);
  return parse(restored);
}

}  // namespace

TEST(RangeFactorWithTransform, Pose2) {
  const auto factor = roundTrip(measurement(jrl::RangeFactorWithTransformPose2Tag, X(0), X(1), 2.0,
                                             serialize<gtsam::Pose2>(gtsam::Pose2(1.0, 0.0, 0.0))));
  const auto typed = boost::dynamic_pointer_cast<gtsam::RangeFactorWithTransform<gtsam::Pose2>>(factor);
  ASSERT_NE(nullptr, typed);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose2());
  values.insert(X(1), gtsam::Pose2(3.0, 0.0, 0.0));
  EXPECT_NEAR(0.0, typed->unwhitenedError(values)(0), 1e-9);
}

TEST(RangeFactorWithTransform, Pose3) {
  const auto factor =
      roundTrip(measurement(jrl::RangeFactorWithTransformPose3Tag, X(0), X(1), 2.0,
                            serialize<gtsam::Pose3>(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)))));
  const auto typed = boost::dynamic_pointer_cast<gtsam::RangeFactorWithTransform<gtsam::Pose3>>(factor);
  ASSERT_NE(nullptr, typed);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3());
  values.insert(X(1), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0, 0.0, 0.0)));
  EXPECT_NEAR(0.0, typed->unwhitenedError(values)(0), 1e-9);
}

TEST(RangeFactorWithTransform, Point2) {
  const auto factor = roundTrip(measurement(jrl::RangeFactorWithTransform2DTag, X(0), L(0), 2.0,
                                             serialize<gtsam::Pose2>(gtsam::Pose2(1.0, 0.0, 0.0))));
  const auto typed = boost::dynamic_pointer_cast<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>>(factor);
  ASSERT_NE(nullptr, typed);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose2());
  values.insert(L(0), gtsam::Point2(3.0, 0.0));
  EXPECT_NEAR(0.0, typed->unwhitenedError(values)(0), 1e-9);
}

TEST(RangeFactorWithTransform, Point3) {
  const auto factor =
      roundTrip(measurement(jrl::RangeFactorWithTransform3DTag, X(0), L(0), 2.0,
                            serialize<gtsam::Pose3>(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)))));
  const auto typed = boost::dynamic_pointer_cast<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>>(factor);
  ASSERT_NE(nullptr, typed);

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3());
  values.insert(L(0), gtsam::Point3(3.0, 0.0, 0.0));
  EXPECT_NEAR(0.0, typed->unwhitenedError(values)(0), 1e-9);
}

TEST(RangeFactorWithTransform, NativeFactorFailsSafely) {
  gtsam::NonlinearFactorGraph graph;
  graph.push_back(gtsam::RangeFactorWithTransform<gtsam::Pose2>(
      X(0), X(1), 2.0, gtsam::noiseModel::Isotropic::Sigma(1, 0.1), gtsam::Pose2(1.0, 0.0, 0.0)));
  const jrl::Entry entry(0, {jrl::RangeFactorWithTransformPose2Tag}, graph);
  const jrl::Writer writer;

  EXPECT_THROW(writer.serializeMeasurements({entry}), std::invalid_argument);
}
