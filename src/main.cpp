#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <Eigen/Dense>

//#include "config.h"
#include "measurement_package.h"
#include "ground_truth_package.h"
#include "fusion_radar_lidar.h"
#include "kalman_filter.h"
#include "tools.h"


// Configuration of filter
// State transition model
const auto state_trans_func = [](float delta_t, const Eigen::VectorXd &x) {
  Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
  // Constant velocity motion model
  F << 1, 0, delta_t, 0,
       0, 1, 0, delta_t,
       0, 0, 1, 0,
       0, 0, 0, 1;
  return std::tuple<Eigen::VectorXd, Eigen::MatrixXd>(F*x, F);
};

// Process noise model
// standard deviation of continuous process noise
const float q_x = 7;
const float q_y = 7; 
const auto process_noise_func = [](float dt, const Eigen::VectorXd &x) {
  Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
  float dt2, dt3, dt4;
  dt2 = dt * dt;
  dt3 = dt2 * dt;
  dt4 = dt3 * dt;

  Q << q_x * dt4 / 4, 0, q_x *dt3 / 2, 0,
       0, q_y * dt4 / 4, 0, q_y * dt3 / 2,
       q_x * dt3 / 2, 0, q_x * dt2, 0,
       0, q_y * dt3 / 2, 0, q_y * dt2;
  return Q;
};
const MotionModel motion_model = MotionModel(state_trans_func, 
					     process_noise_func);

// Observation function, observation matrix, and measurement noise cov
// Lidar
const auto obsev_matrix_func_lidar = [](const Eigen::VectorXd &x) {
  Eigen::MatrixXd H_lidar(2, 4);
  H_lidar << 1, 0, 0, 0,
             0, 1, 0, 0;
  return H_lidar;
};
const Eigen::MatrixXd R_lidar = (Eigen::MatrixXd(2,2) << 0.0225, 0, 
						      0, 0.0225).finished();
const auto lidar_func = [](const Eigen::VectorXd &x) {
  Eigen::VectorXd z(2);
  z << x(0), x(1);
  return z;
};
const ObservModel observ_model_lidar(R_lidar, lidar_func, 
				    obsev_matrix_func_lidar);

// Radar
const auto obsev_matrix_func_radar = [](const Eigen::VectorXd &x) {
  Eigen::MatrixXd H_radar(3, 4);
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  float rho = sqrt(px * px + py * py);
  float rho2 = rho * rho;
  float rho3 = rho2 * rho;

  //check division by zero
  if (rho < 1e-6) {
    std::cerr << "Divide by zero!" << std::endl;
    return H_radar;
  }
  if (abs(rho) < 1e-4) {
    H_radar << 0, 0, 0, 0,
	       0, 0, 0, 0,
	       0, 0, 0, 0;
  } else {
    //compute the Jacobian matrix
    H_radar << px / rho, py / rho, 0, 0,
	       -py / rho2, px / rho2, 0, 0,
	       py * (vx * py - vy * px) / rho3,
	       px * (vy * px - vx * py) / rho3, px / rho, py / rho;
  }
  return H_radar;
};
const Eigen::MatrixXd R_radar = (Eigen::MatrixXd(3, 3) << 0.09, 0, 0,
						          0, 0.09, 0,
							0, 0, 0.09).finished();
const auto radar_func = [](const Eigen::VectorXd &x) {
  Eigen::VectorXd z(3);
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);
  z << sqrt(px*px + py*py),
       atan2(py, px),
       (px*vx + py*vy)/sqrt(px*px + py*py);
  return z;
};
const ObservModel observ_model_radar(R_radar, radar_func, 
				    obsev_matrix_func_radar);

// Initial state error covariance
const Eigen::MatrixXd P_0 = (Eigen::MatrixXd(4,4) << 50, 0, 0, 0,
						     0, 50, 0, 0,
						     0, 0, 100, 0,
				                     0, 0, 0, 100).finished();

// Function prototypes
/**
 * Check whether the input command line arguments are valid
 * If they are invalid, display help info.
 * @param argc Number of comannd line arguments
 * @param argv Array of command line strings
 */
void CheckInputs(int argc, char **argv);

/**
 * Check whether the input and output files are valid
 * @param argc Number of comannd line arguments
 * @param argv Array of command line strings
 */
void CheckFiles(const std::ifstream &input_file, char *input_file_name, 
		const std::ofstream &output_file, char *output_file_name);

/*
 *
 */
bool InitializeEKF(FusionEKF &fusion_EKF, MeasurementPackage &measurement_pack);

int main(int argc, char **argv)
{
  // Load input radar and lidar data
  CheckInputs(argc, argv);
  std::ifstream input_file(argv[1], std::ifstream::in);
  std::ofstream output_file(argv[2], std::ofstream::out);
  CheckFiles(input_file, argv[1], output_file, argv[2]);

  // Go through the input data file and store data to vectors
  std::vector<MeasurementPackage> measurement_package_vec;
  std::vector<GroundTruthPackage> gt_package_vec;
  std::string line;
  while (std::getline(input_file, line)) {
    std::istringstream iss(line);
    
    // Extract sensor type
    std::string sensor_type;
    iss >> sensor_type;

    // Extract measurements
    MeasurementPackage measure_package;
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {
      // Lidar measurements
      float x;
      float y;
      iss >> x;
      iss >> y;
      iss >> timestamp;
      measure_package.sensor_type_ = SensorType::kLIDAR;
      measure_package.timestamp_ = timestamp;
      measure_package.raw_measurements_ = Eigen::VectorXd(2);
      measure_package.raw_measurements_ << x, y;
      measurement_package_vec.push_back(measure_package);
    }
    else if (sensor_type.compare("R") == 0) {
      // Radar measurements
      float rho;
      float phi;
      float rho_dot;
      iss >> rho;
      iss >> phi;
      iss >> rho_dot;
      iss >> timestamp;
      measure_package.sensor_type_ = SensorType::kRADAR;
      measure_package.timestamp_ = timestamp;
      measure_package.raw_measurements_ = Eigen::VectorXd(3);
      measure_package.raw_measurements_ << rho, phi, rho_dot;
      measurement_package_vec.push_back(measure_package);
    }

    // Read ground truth data and store data to vectors
    GroundTruthPackage gt_package;
    float gt_px;
    float gt_py;
    float gt_vx;
    float gt_vy;
    iss >> gt_px;
    iss >> gt_py;
    iss >> gt_vx;
    iss >> gt_vy;
    gt_package.gt_ = Eigen::VectorXd(4);
    gt_package.gt_ << gt_px, gt_py, gt_vx, gt_vy;
    gt_package_vec.push_back(gt_package);
  }

  // Run EKF based fusion
  std::vector<Eigen::VectorXd> estimates;
  std::vector<Eigen::VectorXd> ground_truth;
  FusionEKF fusion_EKF;
  size_t N = measurement_package_vec.size();
  for (size_t i = 0; i < N; ++i) {
    if (!fusion_EKF.IsInitialized()) {
      bool result = InitializeEKF(fusion_EKF, measurement_package_vec[i]);
      if (!result) {
	std::cout << "Skip frame" << i << std::endl;
	continue;
      }
    }
    // Filter data using EKF
    if (measurement_package_vec[i].sensor_type_ == SensorType::kRADAR) {
      fusion_EKF.ProcessMeasurement(measurement_package_vec[i], 
				    observ_model_radar);
    }
    else if (measurement_package_vec[i].sensor_type_ == SensorType::kLIDAR) {
      fusion_EKF.ProcessMeasurement(measurement_package_vec[i], 
				    observ_model_lidar);
    }

    // Output the state estimates
    output_file << fusion_EKF.ekf_.x_(0) << "\t";
    output_file << fusion_EKF.ekf_.x_(1) << "\t";
    output_file << fusion_EKF.ekf_.x_(2) << "\t";
    output_file << fusion_EKF.ekf_.x_(3) << "\t";

    // Output the measurements
    if (measurement_package_vec[i].sensor_type_ == SensorType::kLIDAR) {
      // Output the estimation
      output_file << measurement_package_vec[i].raw_measurements_(0) << "\t";
      output_file << measurement_package_vec[i].raw_measurements_(1) << "\t";
    } else if (measurement_package_vec[i].sensor_type_ == SensorType::kRADAR) {
      // Output the estimation in the cartesian coordinates
      float rho = measurement_package_vec[i].raw_measurements_(0);
      float phi = measurement_package_vec[i].raw_measurements_(1);
      output_file << rho * cos(phi) << "\t"; // p1_meas
      output_file << rho * sin(phi) << "\t"; // ps_meas
    }

    // Output the ground truth packages
    output_file << gt_package_vec[i].gt_(0) << "\t";
    output_file << gt_package_vec[i].gt_(1) << "\t";
    output_file << gt_package_vec[i].gt_(2) << "\t";
    output_file << gt_package_vec[i].gt_(3) << "\n";    

    estimates.push_back(fusion_EKF.ekf_.x_);
    ground_truth.push_back(gt_package_vec[i].gt_);
  }

  // Compute RMSE for accuracy
  std::cout << "\nAccuracy - RMSE:" << std::endl 
       << Tools::CalculateRMSE(estimates, ground_truth) 
       << std::endl;

  // close files
  if (output_file.is_open()) {
    output_file.close();
  }
  if (input_file.is_open()) {
    input_file.close();
  } 

  std::cout << "End of main" << std::endl;
  return 0;
}

/**
 * Check whether the command line arguments are valid
 */
void CheckInputs(int argc, char **argv)
{
  std::string help = "Instructions: ";
  help += argv[0];
  help += " path/to/input_data.txt path/to/result.txt";

  if (argc == 1) {
    std::cerr << help << std::endl;
  }
  else if (argc == 2) {
    std::cerr << "Please provide a result file.\n" << help << std::endl;
  }
  else if (argc == 3) {
    return;
  }
  else if (argc > 3) {
    std::cerr << "Too many arguments.\n" << help << std::endl;
  }
  exit(EXIT_FAILURE);
}

/**
 * Check whether the input and output files are valid
 */
void CheckFiles(const std::ifstream &input_file, char *input_file_name, 
		const std::ofstream &output_file, char *output_file_name)
{
  if (!input_file.is_open()) {
    std::cerr << "Failed to open the input file: " << 
    input_file_name << std::endl;
    exit(EXIT_FAILURE);
  }
  if (!output_file.is_open()) {
    std::cerr << "Failed to open the output file: " << 
    output_file_name << std::endl;
    exit(EXIT_FAILURE);
  }
}

/*
 *
 */
bool InitializeEKF(FusionEKF &fusion_EKF, MeasurementPackage &measurement_pack)
{
  Eigen::VectorXd x_0 = Eigen::VectorXd(4);
  float rho;
  if (measurement_pack.sensor_type_ == SensorType::kRADAR) {
    float phi;
    rho = measurement_pack.raw_measurements_[0];
    phi = measurement_pack.raw_measurements_[1];
    x_0 << rho * cos(phi), rho*sin(phi), 0, 0;
  }
  else if (measurement_pack.sensor_type_ == SensorType::kLIDAR) {
    float px, py;
    px = measurement_pack.raw_measurements_[0];
    py = measurement_pack.raw_measurements_[1];
    rho = sqrt(px * px + py * py);
    x_0 << px, py, 0, 0;
  }
  if (rho < 1e-4) {
    return false;
  }
  fusion_EKF.Init(x_0, P_0, motion_model, measurement_pack.timestamp_);
  return true;
}
