#include <gtsam/navigation/TangentPreintegration.h>

#include <cassert>

class TangentPreintegrationFromComponents final
    : public gtsam::TangentPreintegration {
 public:
  TangentPreintegrationFromComponents(
      const boost::shared_ptr<gtsam::PreintegrationParams>& params,
      const gtsam::Vector9& preintegrated,
      const gtsam::Matrix93& H_biasAcc,
      const gtsam::Matrix93& H_biasOmega,
      const gtsam::imuBias::ConstantBias& biasHat, double deltaTij)
      : TangentPreintegration(params, biasHat) {
    preintegrated_ = preintegrated;
    preintegrated_H_biasAcc_ = H_biasAcc;
    preintegrated_H_biasOmega_ = H_biasOmega;
    deltaTij_ = deltaTij;
  }
};

gtsam::TangentPreintegration makeTangentPreintegration(
    const boost::shared_ptr<gtsam::PreintegrationParams>& params,
    const gtsam::Vector9& preintegrated, const gtsam::Matrix93& H_biasAcc,
    const gtsam::Matrix93& H_biasOmega,
    const gtsam::imuBias::ConstantBias& biasHat, double deltaTij) {
  return TangentPreintegrationFromComponents(
      params, preintegrated, H_biasAcc, H_biasOmega, biasHat, deltaTij);
}

int main() {
  const auto params = gtsam::PreintegrationParams::MakeSharedD(9.81);
  const gtsam::imuBias::ConstantBias biasHat(
      gtsam::Vector3(0.1, -0.2, 0.3), gtsam::Vector3(-0.4, 0.5, -0.6));
  gtsam::TangentPreintegration original(params, biasHat);

  original.integrateMeasurement(gtsam::Vector3(0.3, 0.1, 9.7),
                                gtsam::Vector3(0.01, -0.02, 0.03), 0.1);
  original.integrateMeasurement(gtsam::Vector3(0.2, -0.4, 9.9),
                                gtsam::Vector3(-0.04, 0.05, 0.06), 0.2);

  const gtsam::TangentPreintegration restored = makeTangentPreintegration(
      params, original.preintegrated(), original.preintegrated_H_biasAcc(),
      original.preintegrated_H_biasOmega(), original.biasHat(),
      original.deltaTij());

  assert(original.equals(restored, 1e-12));

  const gtsam::imuBias::ConstantBias anotherBias(
      gtsam::Vector3(-0.7, 0.8, 0.9), gtsam::Vector3(1.0, -1.1, 1.2));
  assert((original.biasCorrectedDelta(anotherBias) -
          restored.biasCorrectedDelta(anotherBias))
             .norm() < 1e-12);
}
