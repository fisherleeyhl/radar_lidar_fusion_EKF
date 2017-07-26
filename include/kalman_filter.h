#ifndef RADAR_LIDAR_FUSION_KALMAN_FILTER_H_
#define RADAR_LIDAR_FUSION_KALMAN_FILTER_H_
#include <functional>
#include <iostream>
#include <Eigen/Dense>

// Function pointer for state transition model 
// that computes the predicted state vector and 
// the state transition matrix based on elapsed time
// and previous state vector
typedef std::function<std::tuple<Eigen::VectorXd, Eigen::MatrixXd>
	(float delta_T, const Eigen::VectorXd &x)> StateTransitionFunc;

// Function pointer that computes the covariance matrix 
// of the process noise using elapsed time and previous state vector
typedef std::function<Eigen::MatrixXd (
	float delta_T, const Eigen::VectorXd &x)> ProcessNoiseCovFunc;

// Function pointer that maps the states to the measurements
typedef std::function<Eigen::VectorXd (const Eigen::VectorXd &x)> 
	SensorFunc;

// Function pointer that computes the observation matrix
typedef std::function<Eigen::MatrixXd (const Eigen::VectorXd &x)> 
	ObservationMatrixFunc;

// Function pointer that combines state transition model and process
// noise covariance
typedef std::tuple<StateTransitionFunc, ProcessNoiseCovFunc> MotionModel;

// Function pointer that combines measurement noise covariance, sensor model
// and observation matrix
typedef std::tuple<Eigen::MatrixXd, SensorFunc, ObservationMatrixFunc> ObservModel;

class KalmanFilter
{
 public:
   // State vector
   Eigen::VectorXd x_;

   // State error covariance matrix
   Eigen::MatrixXd P_;

   // Motion model
   MotionModel motion_model_;

   /**
    * Constructor
    */
   KalmanFilter();

  /**
   * Destructor
   */
  ~KalmanFilter();

   /**
    * Init function initializes a Kalman Filter
    * @param x Initial state
    * @param P Initial state error covariance
    * @param motion_model Motion model by which the state is propagated
    */
   void Init(const Eigen::VectorXd &x, const Eigen::MatrixXd &P, 
             const MotionModel &motion_model);
    
   /**
    * Predict function predicts the state and state covariance
    * using the motion model
    * @param delta_T Time elapsed between previous and current time stamps in second
    */
   void Predict(float delta_T);

   /**
    * Update function performs the measurement update for Kalman Filter
    * @param z Measurement obtained at current time stamp
    * @param observ_model Model that maps the states to the measurements
    */
   void Update(const Eigen::VectorXd &z, const ObservModel &observ_model);
};

#endif // RADAR_LIDAR_FUSION_KALMAN_FILTER_H_
