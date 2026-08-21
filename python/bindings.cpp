#include <gtsam/nonlinear/NonlinearFactor.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "boost_optional.h"
#include "boost_shared_ptr.h"
#include "jrl/Dataset.h"
#include "jrl/DatasetBuilder.h"
#include "jrl/IOMeasurements.h"
#include "jrl/IOValues.h"
#include "jrl/Initialization.h"
#include "jrl/Metrics.h"
#include "jrl/Parser.h"
#include "jrl/Results.h"
#include "jrl/Writer.h"

namespace nb = nanobind;
using namespace jrl;

NB_MODULE(_core, m) {
  m.def("abi_tag", []() { return nb::detail::abi_tag(); }, "Get the ABI tag of the current module.");

  // Import gtsam to ensure that python has access to return types
  nb::module_ gtsam = nb::module_::import_("gtsam");

  m.attr("Rot2Tag") = Rot2Tag;
  m.attr("Pose2Tag") = Pose2Tag;
  m.attr("Rot3Tag") = Rot3Tag;
  m.attr("Pose3Tag") = Pose3Tag;
  m.attr("Point2Tag") = Point2Tag;
  m.attr("Point3Tag") = Point3Tag;
  m.attr("Unit3Tag") = Unit3Tag;
  m.attr("VectorTag") = VectorTag;
  m.attr("ScalarTag") = ScalarTag;
  m.attr("ConstantBiasTag") = ConstantBiasTag;
  m.attr("BearingRangeTag") = BearingRangeTag;

  m.attr("PriorFactorPose2Tag") = PriorFactorPose2Tag;
  m.attr("PriorFactorPose3Tag") = PriorFactorPose3Tag;
  m.attr("BetweenFactorPose2Tag") = BetweenFactorPose2Tag;
  m.attr("BetweenFactorPose3Tag") = BetweenFactorPose3Tag;
  m.attr("RangeFactorPose2Tag") = RangeFactorPose2Tag;
  m.attr("RangeFactorPose3Tag") = RangeFactorPose3Tag;
  m.attr("RangeFactor2DTag") = RangeFactor2DTag;
  m.attr("RangeFactor3DTag") = RangeFactor3DTag;
  m.attr("RangeFactorWithTransformPose2Tag") = RangeFactorWithTransformPose2Tag;
  m.attr("RangeFactorWithTransformPose3Tag") = RangeFactorWithTransformPose3Tag;
  m.attr("RangeFactorWithTransform2DTag") = RangeFactorWithTransform2DTag;
  m.attr("RangeFactorWithTransform3DTag") = RangeFactorWithTransform3DTag;
  m.attr("BearingRangeFactorPose2Tag") = BearingRangeFactorPose2Tag;
  m.attr("BearingRangeFactorPose3Tag") = BearingRangeFactorPose3Tag;
  m.attr("BearingRangeFactor2DTag") = BearingRangeFactor2DTag;
  m.attr("BearingRangeFactor3DTag") = BearingRangeFactor3DTag;
  m.attr("PriorFactorPoint2Tag") = PriorFactorPoint2Tag;
  m.attr("PriorFactorPoint3Tag") = PriorFactorPoint3Tag;
  m.attr("PriorFactorConstantBiasTag") = PriorFactorConstantBiasTag;
  m.attr("BetweenFactorPoint2Tag") = BetweenFactorPoint2Tag;
  m.attr("BetweenFactorPoint3Tag") = BetweenFactorPoint3Tag;
  m.attr("CombinedImuFactorTag") = CombinedImuFactorTag;

  nb::class_<RangeFactorWithTransform<gtsam::Pose2>, gtsam::NonlinearFactor>(m, "RangeFactorWithTransformPose2")
      .def(nb::init<gtsam::Key, gtsam::Key, double, const gtsam::SharedNoiseModel&, const gtsam::Pose2&>(),
           nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("model"), nb::arg("body_T_sensor"))
      .def_prop_ro("body_T_sensor", &RangeFactorWithTransform<gtsam::Pose2>::body_T_sensor);
  nb::class_<RangeFactorWithTransform<gtsam::Pose3>, gtsam::NonlinearFactor>(m, "RangeFactorWithTransformPose3")
      .def(nb::init<gtsam::Key, gtsam::Key, double, const gtsam::SharedNoiseModel&, const gtsam::Pose3&>(),
           nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("model"), nb::arg("body_T_sensor"))
      .def_prop_ro("body_T_sensor", &RangeFactorWithTransform<gtsam::Pose3>::body_T_sensor);
  nb::class_<RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>, gtsam::NonlinearFactor>(
      m, "RangeFactorWithTransform2D")
      .def(nb::init<gtsam::Key, gtsam::Key, double, const gtsam::SharedNoiseModel&, const gtsam::Pose2&>(),
           nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("model"), nb::arg("body_T_sensor"))
      .def_prop_ro("body_T_sensor", &RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>::body_T_sensor);
  nb::class_<RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>, gtsam::NonlinearFactor>(
      m, "RangeFactorWithTransform3D")
      .def(nb::init<gtsam::Key, gtsam::Key, double, const gtsam::SharedNoiseModel&, const gtsam::Pose3&>(),
           nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("model"), nb::arg("body_T_sensor"))
      .def_prop_ro("body_T_sensor", &RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>::body_T_sensor);

  /**
   * ########     ###    ########    ###     ######  ######## ########
   * ##     ##   ## ##      ##      ## ##   ##    ## ##          ##
   * ##     ##  ##   ##     ##     ##   ##  ##       ##          ##
   * ##     ## ##     ##    ##    ##     ##  ######  ######      ##
   * ##     ## #########    ##    ##     ## ##       ##          ##
   * ##     ## ##     ##    ##    ##     ## ##    ## ##          ##
   * ########  ##     ##    ##    ##     ##  ######  ########    ##
   */
  /**********************************************************************************************************************/
  nb::class_<Entry>(m, "Entry")
      .def(nb::init<uint64_t &, std::vector<std::string> &, gtsam::NonlinearFactorGraph &>(), nb::arg("stamp"),
           nb::arg("measurement_types"), nb::arg("measurements"))
      .def_rw("stamp", &Entry::stamp)
      .def_rw("measurement_types", &Entry::measurement_types)
      .def_rw("measurements", &Entry::measurements)
      .def("filtered", &Entry::filtered)
      .def_static("KeepTypes", &Entry::KeepTypes)
      .def_static("RemoveTypes", &Entry::RemoveTypes)
      .def("__getstate__",
           [](const Entry &entry) { return nb::make_tuple(entry.stamp, entry.measurement_types, entry.measurements); })
      .def("__setstate__", [](Entry &entry, nb::tuple tup) {
        new (&entry) Entry(nb::cast<uint64_t>(tup[0]), nb::cast<std::vector<std::string>>(tup[1]),
                           nb::cast<gtsam::NonlinearFactorGraph>(tup[2]));
      });

  /**********************************************************************************************************************/
  nb::class_<TypedValues>(m, "TypedValues")
      .def(nb::init<gtsam::Values &, ValueTypes &>())
      .def_rw("values", &TypedValues::values)
      .def_rw("types", &TypedValues::types)
      .def("__getstate__",
           [](const TypedValues &typed_values) { return nb::make_tuple(typed_values.values, typed_values.types); })
      .def("__setstate__", [](TypedValues &tv, nb::tuple tup) {
        new (&tv) TypedValues(nb::cast<gtsam::Values>(tup[0]), nb::cast<ValueTypes>(tup[1]));
      });

  /**********************************************************************************************************************/
  nb::class_<Dataset>(m, "Dataset")
      .def(nb::init<const std::string &, std::vector<char> &, std::map<char, std::vector<Entry>> &,
                    boost::optional<std::map<char, TypedValues>> &, boost::optional<std::map<char, TypedValues>> &,
                    boost::optional<std::map<char, std::set<FactorId>>> &,
                    boost::optional<std::map<char, std::set<FactorId>>> &>())
      .def("name", &Dataset::name)
      .def("robots", &Dataset::robots)
      .def("measurements", &Dataset::measurements)
      .def("factorGraph", &Dataset::factorGraph)
      .def("groundTruth", &Dataset::groundTruth)
      .def("groundTruthWithTypes", &Dataset::groundTruthWithTypes)
      .def("containsGroundTruth", &Dataset::containsGroundTruth)
      .def("initialization", &Dataset::initialization)
      .def("initializationWithTypes", &Dataset::initializationWithTypes)
      .def("containsInitialization", &Dataset::containsInitialization)
      .def("potentialOutlierFactors", &Dataset::potentialOutlierFactors)
      .def("containsPotentialOutlierFactors", &Dataset::containsPotentialOutlierFactors)
      .def("outlierFactors", &Dataset::outlierFactors)
      .def("containsOutlierFactors", &Dataset::containsOutlierFactors)
      .def("__getstate__",
           [](const Dataset &dataset) {
             std::map<char, std::vector<Entry>> measurements;
             boost::optional<std::map<char, TypedValues>> ground_truth = boost::none;
             boost::optional<std::map<char, TypedValues>> initialization = boost::none;
             boost::optional<std::map<char, std::set<FactorId>>> potential_outlier_factors = boost::none;
             boost::optional<std::map<char, std::set<FactorId>>> outlier_factors = boost::none;

             if (dataset.containsGroundTruth()) {
               ground_truth = std::map<char, TypedValues>();
             }
             if (dataset.containsInitialization()) {
               initialization = std::map<char, TypedValues>();
             }
             if (dataset.containsPotentialOutlierFactors()) {
               potential_outlier_factors = std::map<char, std::set<FactorId>>();
             }
             if (dataset.containsOutlierFactors()) {
               outlier_factors = std::map<char, std::set<FactorId>>();
             }

             for (auto &rid : dataset.robots()) {
               measurements[rid] = dataset.measurements(rid);
               if (ground_truth) (*ground_truth)[rid] = dataset.groundTruthWithTypes(rid);
               if (initialization) (*initialization)[rid] = dataset.initializationWithTypes(rid);
               if (potential_outlier_factors) (*potential_outlier_factors)[rid] = dataset.potentialOutlierFactors(rid);
               if (outlier_factors) (*outlier_factors)[rid] = dataset.outlierFactors(rid);
             }

             return nb::make_tuple(dataset.name(), dataset.robots(), measurements, ground_truth, initialization,
                                   potential_outlier_factors, outlier_factors);
           })
      .def("__setstate__", [](Dataset &dataset, nb::tuple tup) {
        new (&dataset) Dataset(nb::cast<std::string>(tup[0]), nb::cast<std::vector<char>>(tup[1]),
                               nb::cast<std::map<char, std::vector<Entry>>>(tup[2]),
                               nb::cast<boost::optional<std::map<char, TypedValues>>>(tup[3]),
                               nb::cast<boost::optional<std::map<char, TypedValues>>>(tup[4]),
                               nb::cast<boost::optional<std::map<char, std::set<FactorId>>>>(tup[5]),
                               nb::cast<boost::optional<std::map<char, std::set<FactorId>>>>(tup[6]));
      });

  /**********************************************************************************************************************/
  nb::class_<DatasetBuilder>(m, "DatasetBuilder")
      .def(nb::init<const std::string &, std::vector<char> &>())
      .def("addEntry", &DatasetBuilder::addEntry, nb::arg("robot"), nb::arg("stamp"), nb::arg("measurements"),
           nb::arg("measurement_types"), nb::arg("initialization") = nb::none(), nb::arg("groundtruth") = nb::none())
      .def("addGroundTruth", &DatasetBuilder::addGroundTruth, nb::arg("robot"), nb::arg("groundtruth"))
      .def("addInitialization", &DatasetBuilder::addInitialization, nb::arg("robot"), nb::arg("initialization"))
      .def("setPotentialOutlierFactors", &DatasetBuilder::setPotentialOutlierFactors, nb::arg("robot"),
           nb::arg("potential_outlier_factors"))
      .def("setOutlierFactors", &DatasetBuilder::setOutlierFactors, nb::arg("robot"), nb::arg("outlier_factors"))
      .def("build", &DatasetBuilder::build);

  /**
   * ########  ########  ######  ##     ## ##       ########  ######
   * ##     ## ##       ##    ## ##     ## ##          ##    ##    ##
   * ##     ## ##       ##       ##     ## ##          ##    ##
   * ########  ######    ######  ##     ## ##          ##     ######
   * ##   ##   ##             ## ##     ## ##          ##          ##
   * ##    ##  ##       ##    ## ##     ## ##          ##    ##    ##
   * ##     ## ########  ######   #######  ########    ##     ######
   */

  /**********************************************************************************************************************/
  nb::class_<Results>(m, "Results")
      .def(nb::init<const std::string &, const std::string &, std::vector<char> &, std::map<char, TypedValues> &>())
      .def_rw("dataset_name", &Results::dataset_name)
      .def_rw("method_name", &Results::method_name)
      .def_rw("robots", &Results::robots)
      .def_rw("robot_solutions", &Results::robot_solutions)
      .def_rw("robot_outliers", &Results::robot_outliers)
      .def("__getstate__",
           [](const Results &results) {
             return nb::make_tuple(results.dataset_name, results.method_name, results.robots, results.robot_solutions,
                                   results.robot_outliers);
           })
      .def("__setstate__", [](Results &results, nb::tuple tup) {
        new (&results) Results(nb::cast<std::string>(tup[0]), nb::cast<std::string>(tup[1]),
                               nb::cast<std::vector<char>>(tup[2]), nb::cast<std::map<char, TypedValues>>(tup[3]),
                               nb::cast<boost::optional<std::map<char, std::set<FactorId>>>>(tup[4]));
      });

  /**********************************************************************************************************************/
  nb::class_<Parser>(m, "Parser")
      .def(nb::init<>())
      .def("parseDataset", &Parser::parseDataset)
      .def("parseResults", &Parser::parseResults)
      .def("parseMetricSummary", &Parser::parseMetricSummary);

  /**********************************************************************************************************************/
  nb::class_<Writer>(m, "Writer")
      .def(nb::init<>())
      .def("writeDataset", &Writer::writeDataset)
      .def("writeResults", &Writer::writeResults)
      .def("writeMetricSummary", &Writer::writeMetricSummary);

  /**
   * ##     ## ######## ######## ########  ####  ######   ######
   * ###   ### ##          ##    ##     ##  ##  ##    ## ##    ##
   * #### #### ##          ##    ##     ##  ##  ##       ##
   * ## ### ## ######      ##    ########   ##  ##        ######
   * ##     ## ##          ##    ##   ##    ##  ##             ##
   * ##     ## ##          ##    ##    ##   ##  ##    ## ##    ##
   * ##     ## ########    ##    ##     ## ####  ######   ######
   */

  /**********************************************************************************************************************/
  nb::class_<MetricSummary>(m, "MetricSummary")
      .def(nb::init<>())
      .def_rw("robots", &MetricSummary::robots)
      .def_rw("dataset_name", &MetricSummary::dataset_name)
      .def_rw("method_name", &MetricSummary::method_name)
      .def_rw("robot_ate", &MetricSummary::robot_ate)
      .def_rw("total_ate", &MetricSummary::total_ate)
      .def_rw("joint_aligned_ate", &MetricSummary::joint_aligned_ate)
      .def_rw("sve", &MetricSummary::sve)
      .def_rw("mean_residual", &MetricSummary::mean_residual)
      .def_rw("robot_precision_recall", &MetricSummary::robot_precision_recall)
      .def_rw("precision_recall", &MetricSummary::precision_recall);

  /**********************************************************************************************************************/
  m.def("computeMetricSummaryPoint2", &metrics::computeMetricSummary<gtsam::Point2>, nb::rv_policy::copy,
        nb::arg("dataset"), nb::arg("results"), nb::arg("ate_align") = true, nb::arg("ate_align_with_scale") = false,
        nb::arg("ate_include_shared_variables") = true, nb::arg("step_idx") = nb::none());
  m.def("computeMetricSummaryPoint3", &metrics::computeMetricSummary<gtsam::Point3>, nb::rv_policy::copy,
        nb::arg("dataset"), nb::arg("results"), nb::arg("ate_align") = true, nb::arg("ate_align_with_scale") = false,
        nb::arg("ate_include_shared_variables") = true, nb::arg("step_idx") = nb::none());
  m.def("computeMetricSummaryPose2", &metrics::computeMetricSummary<gtsam::Pose2>, nb::rv_policy::copy,
        nb::arg("dataset"), nb::arg("results"), nb::arg("ate_align") = true, nb::arg("ate_align_with_scale") = false,
        nb::arg("ate_include_shared_variables") = true, nb::arg("step_idx") = nb::none());
  m.def("computeMetricSummaryPose3", &metrics::computeMetricSummary<gtsam::Pose3>, nb::rv_policy::copy,
        nb::arg("dataset"), nb::arg("results"), nb::arg("ate_align") = true, nb::arg("ate_align_with_scale") = false,
        nb::arg("ate_include_shared_variables") = true, nb::arg("step_idx") = nb::none());

  /**********************************************************************************************************************/
  m.def("computeMeanResidual", &metrics::computeMeanResidual, nb::rv_policy::copy, nb::arg("dataset"),
        nb::arg("results"), nb::arg("step_idx") = nb::none());

  /**********************************************************************************************************************/
  m.def("computeSVEPoint2", &metrics::computeSVE<gtsam::Point2>, nb::rv_policy::copy, nb::arg("results"));
  m.def("computeSVEPoint3", &metrics::computeSVE<gtsam::Point3>, nb::rv_policy::copy, nb::arg("results"));
  m.def("computeSVEPose2", &metrics::computeSVE<gtsam::Pose2>, nb::rv_policy::copy, nb::arg("results"));
  m.def("computeSVEPose3", &metrics::computeSVE<gtsam::Pose3>, nb::rv_policy::copy, nb::arg("results"));

  /**********************************************************************************************************************/
  m.def("computeATEPoint2", &metrics::computeATE<gtsam::Point2>, nb::rv_policy::copy, nb::arg("rid"),
        nb::arg("dataset"), nb::arg("results"), nb::arg("align") = true, nb::arg("align_with_scale") = false,
        nb::arg("allow_partial_results") = false, nb::arg("include_shared_variables") = false);
  m.def("computeATEPoint3", &metrics::computeATE<gtsam::Point3>, nb::rv_policy::copy, nb::arg("rid"),
        nb::arg("dataset"), nb::arg("results"), nb::arg("align") = true, nb::arg("align_with_scale") = false,
        nb::arg("allow_partial_results") = false, nb::arg("include_shared_variables") = false);
  m.def("computeATEPose2", &metrics::computeATE<gtsam::Pose2>, nb::rv_policy::copy, nb::arg("rid"), nb::arg("dataset"),
        nb::arg("results"), nb::arg("align") = true, nb::arg("align_with_scale") = false,
        nb::arg("allow_partial_results") = false, nb::arg("include_shared_variables") = false);
  m.def("computeATEPose3", &metrics::computeATE<gtsam::Pose3>, nb::rv_policy::copy, nb::arg("rid"), nb::arg("dataset"),
        nb::arg("results"), nb::arg("align") = true, nb::arg("align_with_scale") = false,
        nb::arg("allow_partial_results") = false, nb::arg("include_shared_variables") = false);

  /**
   * #### ##    ## #### ######## ####    ###    ##       #### ########    ###    ######## ####  #######  ##    ##
   *  ##  ###   ##  ##     ##     ##    ## ##   ##        ##       ##    ## ##      ##     ##  ##     ## ###   ##
   *  ##  ####  ##  ##     ##     ##   ##   ##  ##        ##      ##    ##   ##     ##     ##  ##     ## ####  ##
   *  ##  ## ## ##  ##     ##     ##  ##     ## ##        ##     ##    ##     ##    ##     ##  ##     ## ## ## ##
   *  ##  ##  ####  ##     ##     ##  ######### ##        ##    ##     #########    ##     ##  ##     ## ##  ####
   *  ##  ##   ###  ##     ##     ##  ##     ## ##        ##   ##      ##     ##    ##     ##  ##     ## ##   ###
   * #### ##    ## ####    ##    #### ##     ## ######## #### ######## ##     ##    ##    ####  #######  ##    ##
   */
  nb::class_<Initializer>(m, "Initializer").def(nb::init<>()).def("initialization", &Initializer::initialization);
  /**
   *    ###    ##       ####  ######   ##    ## ##     ## ######## ##    ## ########
   *   ## ##   ##        ##  ##    ##  ###   ## ###   ### ##       ###   ##    ##
   *  ##   ##  ##        ##  ##        ####  ## #### #### ##       ####  ##    ##
   * ##     ## ##        ##  ##   #### ## ## ## ## ### ## ######   ## ## ##    ##
   * ######### ##        ##  ##    ##  ##  #### ##     ## ##       ##  ####    ##
   * ##     ## ##        ##  ##    ##  ##   ### ##     ## ##       ##   ###    ##
   * ##     ## ######## ####  ######   ##    ## ##     ## ######## ##    ##    ##
   */
  m.def("alignPose2", &alignment::align<gtsam::Pose2>, nb::rv_policy::copy, nb::arg("estimate_trajectory"),
        nb::arg("reference_trajectory"), nb::arg("align_with_scale") = false);
  m.def("alignPose3", &alignment::align<gtsam::Pose3>, nb::rv_policy::copy, nb::arg("estimate_trajectory"),
        nb::arg("reference_trajectory"), nb::arg("align_with_scale") = false);
}
