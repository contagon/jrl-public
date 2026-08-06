#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <jrl/IOMeasurements.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

TEST(CombinedImuFactor, RoundTrip) {
  const auto params = gtsam::PreintegrationCombinedParams::MakeSharedD(9.81);
  params->setAccelerometerCovariance(0.01 * gtsam::I_3x3);
  params->setGyroscopeCovariance(0.02 * gtsam::I_3x3);
  params->setIntegrationCovariance(0.03 * gtsam::I_3x3);
  params->setBiasAccCovariance(0.04 * gtsam::I_3x3);
  params->setBiasOmegaCovariance(0.05 * gtsam::I_3x3);
  params->setBiasAccOmegaInit(0.06 * gtsam::I_6x6);
  params->setUse2ndOrderCoriolis(true);
  params->setOmegaCoriolis(gtsam::Vector3(0.01, -0.02, 0.03));
  params->setBodyPSensor(gtsam::Pose3(gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3), gtsam::Point3(0.4, -0.5, 0.6)));

  const gtsam::imuBias::ConstantBias bias_hat(gtsam::Vector3(0.1, -0.2, 0.3), gtsam::Vector3(-0.4, 0.5, -0.6));
  gtsam::PreintegratedCombinedMeasurements pim(params, bias_hat);
  pim.integrateMeasurement(gtsam::Vector3(0.3, 0.1, 9.7), gtsam::Vector3(0.01, -0.02, 0.03), 0.1);
  pim.integrateMeasurement(gtsam::Vector3(0.2, -0.4, 9.9), gtsam::Vector3(-0.04, 0.05, 0.06), 0.2);

  const gtsam::CombinedImuFactor original(X(0), V(0), X(1), V(1), B(0), B(1), pim);
  gtsam::NonlinearFactorGraph graph;
  graph.push_back(original);
  const jrl::Entry entry(42, {jrl::CombinedImuFactorTag}, graph);

  const jrl::Writer writer;
  const jrl::Parser parser;
  const std::vector<jrl::Entry> parsed = parser.parseMeasurements(writer.serializeMeasurements({entry}));

  ASSERT_EQ(parsed.size(), 1);
  ASSERT_EQ(parsed[0].measurements.size(), 1);
  const auto restored = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(parsed[0].measurements.at(0));
  ASSERT_TRUE(restored);
  EXPECT_TRUE(original.equals(*restored, 1e-12));
  EXPECT_TRUE(pim.equals(restored->preintegratedMeasurements(), 1e-12));

  const gtsam::imuBias::ConstantBias another_bias(gtsam::Vector3(-0.7, 0.8, 0.9), gtsam::Vector3(1.0, -1.1, 1.2));
  EXPECT_TRUE(pim.biasCorrectedDelta(another_bias)
                  .isApprox(restored->preintegratedMeasurements().biasCorrectedDelta(another_bias), 1e-12));
}
