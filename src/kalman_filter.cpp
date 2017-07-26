#include "kalman_filter.h"
#include "tools.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(const Eigen::VectorXd &x, const Eigen::MatrixXd &P, 
	    		const MotionModel &motion_model)
{
  x_ = x;
  P_ = P;
  motion_model_ = motion_model;
}
 
/**
 * Function pointers are used to specify the model for state propagation
 * and the function for process noise covariance computation
 */   
void KalmanFilter::Predict(float delta_T)
{ 
  Eigen::MatrixXd F;
  StateTransitionFunc state_trans_func;
  ProcessNoiseCovFunc process_noise_cov_func;

  // State vector prediction
  std::tie(state_trans_func, process_noise_cov_func) = motion_model_;
  std::tie(x_, F) = state_trans_func(delta_T, x_);
    
  // Error covariance matrix prediction
  auto Q = process_noise_cov_func(delta_T, x_);
  P_ = F * P_ * F.transpose() + Q;
}

/**
 * Function pointers are used to specify the model that maps the states to 
 * the measurements and the functions for the computation of the observation 
 * matrix and the measurement noise covariance
 */  
void KalmanFilter::Update(const Eigen::VectorXd &z, 
			  const ObservModel &observ_model)
{
  Eigen::MatrixXd R;
  Eigen::MatrixXd H;
  Eigen::VectorXd z_pred;
  SensorFunc h_func;
  ObservationMatrixFunc H_func;

  std::tie(R, h_func, H_func) = observ_model;

  // get predicted measurements
  z_pred = h_func(x_);
  if (H_func == NULL) {
    H = Tools::ComputeJacobian(h_func, x_);
  }
  else {
    H = H_func(x_);
  }

  // measurement update
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H) * P_;
}
