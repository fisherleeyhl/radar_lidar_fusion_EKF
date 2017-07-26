#ifndef RADAR_LIDAR_FUSION_MEASUREMENT_PACKAGE_H_
#define RADAR_LIDAR_FUSION_MEASUREMENT_PACKAGE_H_
#include <stdint.h>
#include <Eigen/Dense>

enum SensorType
{
  kLIDAR,
  kRADAR
};

class MeasurementPackage
{
 public:
   int64_t timestamp_;
   SensorType sensor_type_;
   Eigen::VectorXd raw_measurements_;
};

#endif // RADAR_LIDAR_FUSION_MEASUREMENT_PACKAGE_H_
