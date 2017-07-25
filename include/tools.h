#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * A helper method to calculate RMSE.
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



#endif /* TOOLS_H_ */
