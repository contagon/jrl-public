#include "jrl/IOMeasurements.h"

#include <gtsam/navigation/CombinedImuFactor.h>

#include <nlohmann/json.hpp>
#include <stdexcept>

#include "jrl/IOValues.h"
using json = nlohmann::json;

namespace jrl {
namespace io_measurements {

template <typename Derived>
json serializeMatrix(const Eigen::MatrixBase<Derived>& matrix) {
  return std::vector<double>(matrix.derived().data(), matrix.derived().data() + matrix.size());
}

gtsam::Matrix parseMatrix(const json& input_json, Eigen::Index rows, Eigen::Index columns) {
  const std::vector<double> values = input_json.get<std::vector<double>>();
  if (values.size() != static_cast<size_t>(rows * columns)) {
    throw std::invalid_argument("Matrix has an unexpected number of elements");
  }
  return Eigen::Map<const gtsam::Matrix>(values.data(), rows, columns);
}

class TangentPreintegrationFromComponents : public gtsam::TangentPreintegration {
 public:
  TangentPreintegrationFromComponents(const boost::shared_ptr<gtsam::PreintegrationParams>& params,
                                      const gtsam::Vector9& preintegrated, const gtsam::Matrix93& h_bias_acc,
                                      const gtsam::Matrix93& h_bias_omega, const gtsam::imuBias::ConstantBias& bias_hat,
                                      double delta_t)
      : TangentPreintegration(params, bias_hat) {
    preintegrated_ = preintegrated;
    preintegrated_H_biasAcc_ = h_bias_acc;
    preintegrated_H_biasOmega_ = h_bias_omega;
    deltaTij_ = delta_t;
  }
};

/**********************************************************************************************************************/
gtsam::Matrix parseCovariance(json input_json, int d) {
  auto v = input_json.get<std::vector<double>>();
  gtsam::Matrix m = Eigen::Map<gtsam::Matrix>(v.data(), d, d);
  return m;
}

json serializeCovariance(gtsam::Matrix covariance) {
  std::vector<double> vec(covariance.data(), covariance.data() + covariance.rows() * covariance.cols());
  return json(vec);
}

/**********************************************************************************************************************/
gtsam::NonlinearFactor::shared_ptr parseCombinedImuFactor(const json& input_json) {
  const json& pim_json = input_json.at("pim");
  const json& params_json = pim_json.at("params");

  const auto params =
      boost::make_shared<gtsam::PreintegrationCombinedParams>(parseMatrix(params_json.at("gravity"), 3, 1));
  params->setAccelerometerCovariance(parseMatrix(params_json.at("accelerometer_covariance"), 3, 3));
  params->setGyroscopeCovariance(parseMatrix(params_json.at("gyroscope_covariance"), 3, 3));
  params->setIntegrationCovariance(parseMatrix(params_json.at("integration_covariance"), 3, 3));
  params->setBiasAccCovariance(parseMatrix(params_json.at("bias_acc_covariance"), 3, 3));
  params->setBiasOmegaCovariance(parseMatrix(params_json.at("bias_omega_covariance"), 3, 3));
  params->setBiasAccOmegaInit(parseMatrix(params_json.at("bias_acc_omega_init"), 6, 6));
  params->setUse2ndOrderCoriolis(params_json.at("use_2nd_order_coriolis").get<bool>());
  if (params_json.contains("omega_coriolis")) {
    params->setOmegaCoriolis(parseMatrix(params_json.at("omega_coriolis"), 3, 1));
  }
  if (params_json.contains("body_p_sensor")) {
    params->setBodyPSensor(io_values::parse<gtsam::Pose3>(params_json.at("body_p_sensor")));
  }

  const gtsam::Vector6 bias_hat_vec = parseMatrix(pim_json.at("bias_hat"), 6, 1);
  const gtsam::imuBias::ConstantBias bias_hat(bias_hat_vec.head<3>(), bias_hat_vec.tail<3>());
  const gtsam::Vector9 preintegrated = parseMatrix(pim_json.at("preintegrated"), 9, 1);
  const gtsam::Matrix93 h_bias_acc = parseMatrix(pim_json.at("h_bias_acc"), 9, 3);
  const gtsam::Matrix93 h_bias_omega = parseMatrix(pim_json.at("h_bias_omega"), 9, 3);
  const double delta_t = pim_json.at("delta_t").get<double>();
  const gtsam::Matrix covariance = parseMatrix(pim_json.at("covariance"), 15, 15);

  const TangentPreintegrationFromComponents tangent(boost::static_pointer_cast<gtsam::PreintegrationParams>(params),
                                                    preintegrated, h_bias_acc, h_bias_omega, bias_hat, delta_t);
  const gtsam::PreintegratedCombinedMeasurements pim(tangent, covariance);

  return boost::make_shared<gtsam::CombinedImuFactor>(
      input_json.at("pose_i").get<gtsam::Key>(), input_json.at("vel_i").get<gtsam::Key>(),
      input_json.at("pose_j").get<gtsam::Key>(), input_json.at("vel_j").get<gtsam::Key>(),
      input_json.at("bias_i").get<gtsam::Key>(), input_json.at("bias_j").get<gtsam::Key>(), pim);
}

/**********************************************************************************************************************/
json serializeCombinedImuFactor(gtsam::NonlinearFactor::shared_ptr& factor) {
  const auto combined = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
  const auto& pim = combined->preintegratedMeasurements();
  const auto& params = pim.p();

  json params_json;
  params_json["gravity"] = serializeMatrix(params.getGravity());
  params_json["accelerometer_covariance"] = serializeMatrix(params.getAccelerometerCovariance());
  params_json["gyroscope_covariance"] = serializeMatrix(params.getGyroscopeCovariance());
  params_json["integration_covariance"] = serializeMatrix(params.getIntegrationCovariance());
  params_json["bias_acc_covariance"] = serializeMatrix(params.getBiasAccCovariance());
  params_json["bias_omega_covariance"] = serializeMatrix(params.getBiasOmegaCovariance());
  params_json["bias_acc_omega_init"] = serializeMatrix(params.getBiasAccOmegaInit());
  params_json["use_2nd_order_coriolis"] = params.getUse2ndOrderCoriolis();
  if (const auto omega_coriolis = params.getOmegaCoriolis()) {
    params_json["omega_coriolis"] = serializeMatrix(*omega_coriolis);
  }
  if (const auto body_p_sensor = params.getBodyPSensor()) {
    params_json["body_p_sensor"] = io_values::serialize<gtsam::Pose3>(*body_p_sensor);
  }

  json pim_json;
  pim_json["params"] = params_json;
  pim_json["covariance"] = serializeMatrix(pim.preintMeasCov());
  pim_json["preintegrated"] = serializeMatrix(pim.preintegrated());
  pim_json["h_bias_acc"] = serializeMatrix(pim.preintegrated_H_biasAcc());
  pim_json["h_bias_omega"] = serializeMatrix(pim.preintegrated_H_biasOmega());
  pim_json["bias_hat"] = serializeMatrix(pim.biasHat().vector());
  pim_json["delta_t"] = pim.deltaTij();

  json output;
  output["type"] = CombinedImuFactorTag;
  output["pose_i"] = combined->keys()[0];
  output["vel_i"] = combined->keys()[1];
  output["pose_j"] = combined->keys()[2];
  output["vel_j"] = combined->keys()[3];
  output["bias_i"] = combined->keys()[4];
  output["bias_j"] = combined->keys()[5];
  output["pim"] = pim_json;
  return output;
}

}  // namespace io_measurements
}  // namespace jrl
