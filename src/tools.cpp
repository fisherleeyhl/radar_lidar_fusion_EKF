#include <iostream>
#include "tools.h"

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4), dx(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if(estimations.size() == 0)
  {
    cerr << "Need at least one estimate, boss!" << endl;
    return rmse;
  }else if(estimations.size() != ground_truth.size())
  {
    cerr << "estimations.size() != ground_truth.size()" <<endl;
    return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    dx = (estimations[i] - ground_truth[i]);
    rmse += dx.cwiseAbs2();
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.cwiseSqrt();

  //return the result
  return rmse;
}