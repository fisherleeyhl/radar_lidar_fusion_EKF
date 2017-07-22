#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <Eigen/Dense>

//#include "fusion_radar_lidar.h"
//#include "ground_truth_package.h"
//#include "measurement_package.h"
#include "kalman_filter.h"
#include "tools.h"

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

int main(int argc, char **argv)
{
  // Load input radar and lidar data
  CheckInputs(argc, argv);
  std::ifstream input_file(argv[1], std::ifstream::in);
  std::ofstream output_file(argv[2], std::ofstream::out);
  CheckFiles(input_file, argv[1], output_file, argv[2]);

  // Go through the input data file
  std::string line;
  while (std::getline(input_file, line)) {
    std::istringstream iss(line);
    
    // Extract sensor type
    std::string sensor_type;
    iss >> sensor_type;

    int64_t time_stamp;
    if (sensor_type.compare("L") == 0) {
      // Lidar measurements
      float x;
      float y;
      iss >> x;
      iss >> y;
      iss >> time_stamp;
    }
    else if (sensor_type.compare("R") == 0) {
      // Radar measurements
      float rho;
      float phi;
      float rho_dot;
      iss >> rho;
      iss >> phi;
      iss >> rho_dot;
      iss >> time_stamp;
    }

    // Read ground truth data
    float gt_px;
    float gt_py;
    float gt_vx;
    float gt_vy;
    iss >> gt_px;
    iss >> gt_py;
    iss >> gt_vx;
    iss >> gt_vy; 
    
    std::cout << sensor_type << std::endl;
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
