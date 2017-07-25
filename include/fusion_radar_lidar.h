#ifndef RADAR_LIDAR_FUSION_FUSION_RADAR_LIDAR_H_
#define RADAR_LIDAR_FUSION_FUSION_RADAR_LIDAR_H_

#include <stdint.h>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include "measurement_package.h"
#include "kalman_filter.h"
#include "tools.h"


class FusionEKF
{
 public:
   /**
    * Constructor
    */
   FusionEKF();

   /**
    * Destructor
    */
   virtual ~FusionEKF();

   /**
    * This function initializes the extended Kalman Filter
    * @param x0 Initial state vector
    * @param P0 Initial state error covariance
    * @param motion_model Motion model by which the state is propagated
    * @param timestamp Time stamp when the data starts being processed
    */
   void Init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0,
	     const MotionModel &motion_model, int64_t timestamp);

   /**
    * This function processes measurement using Kalman Filter
    * @param measurement_pack The package contains all the measurements
    */
   void ProcessMeasurement(const MeasurementPackage &measurement_pack,
			   const ObservModel &observ_model);

   /**
    * This function checks whether the Kalman Filter was initilized
    */
   bool IsInitialized();
   
   KalmanFilter ekf_;

 private:
   bool is_initialized_;
   int64_t previous_timestamp_;
};

#endif // RADAR_LIDAR_FUSION_FUSION_RADAR_LIDAR_H_


