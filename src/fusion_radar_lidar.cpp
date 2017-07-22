#include "fusion_radar_lidar.h"

FusionEKF::FusionEKF()
{
  is_initialized_ = false;
  previous_timestamp_ = 0;
}

FusionEKF::~FusionEKF() {}
