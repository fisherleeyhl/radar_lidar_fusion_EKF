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
    * 
    */

 private:
   bool is_initialized_;
   int64_t previous_timestamp_;
};




#endif

