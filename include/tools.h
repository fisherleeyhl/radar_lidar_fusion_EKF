#ifndef RADAR_LIDAR_FUSION_TOOLS_H_
#define RADAR_LIDAR_FUSION_TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * CalculateRMSE function calculates the RMSE of state estimates
  * @param estimation State estimates
  * @param ground_truth Ground truth
  */
  static VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
  template<class Func>
  inline static MatrixXd ComputeJacobian(Func f, const VectorXd& x, const float h=1e-4)
  {
    int m, n;
    MatrixXd jac;

    n = x.size();
    MatrixXd I = MatrixXd::Identity(n, n)*h;
    m = f(x).size();
    jac = MatrixXd(m, n);

    for(auto i=0; i<x.size(); i++)
    {
      auto step = I.col(i);
      jac.col(i) = (f(x+step) - f(x-step))/(2*h);
    }
    return jac;
  };
};

#endif /* RADAR_LIDAR_FUSION_TOOLS_H_ */
