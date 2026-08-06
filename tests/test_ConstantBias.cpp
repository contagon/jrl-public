#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <jrl/IOValues.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::B;

TEST(ConstantBias, ValuesRoundTrip) {
  const gtsam::imuBias::ConstantBias original(gtsam::Vector3(0.1, -0.2, 0.3), gtsam::Vector3(-0.4, 0.5, -0.6));
  gtsam::Values values;
  values.insert(B(0), original);
  const jrl::TypedValues typed_values(values, {{B(0), jrl::ConstantBiasTag}});

  const jrl::Writer writer;
  const jrl::Parser parser;
  const jrl::TypedValues parsed = parser.parseValues(writer.serializeValues(typed_values));

  EXPECT_EQ(parsed.types.at(B(0)), jrl::ConstantBiasTag);
  EXPECT_TRUE(original.equals(parsed.values.at<gtsam::imuBias::ConstantBias>(B(0)), 1e-12));
}
