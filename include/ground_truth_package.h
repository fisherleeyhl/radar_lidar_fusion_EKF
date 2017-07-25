#ifndef RADAR_LIDAR_FUSION_GROUND_TRUTH_PACKAGE_H_
#define RADAR_LIDAR_FUSION_GROUND_TRUTH_PACKAGE_H_
#include <stdint.h>
#include <Eigen/Dense>
#include "measurement_package.h"

class GroundTruthPackage
{
 public:
   int64_t timestamp_;
   SensorType sensor_type_;
   Eigen::VectorXd gt_;
};

#endif // RADAR_LIDAR_FUSION_GROUND_TRUTH_PACKAGE_H_
