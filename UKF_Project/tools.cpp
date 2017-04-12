#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    assert(estimations.size()>0);

    //  * the estimation vector size should equal ground truth vector size
    assert(estimations.size() == ground_truth.size());

    //accumulate squared residuals
    VectorXd squared_residuals(estimations.size());
    for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd residual = estimations[i]-ground_truth[i];

        squared_residuals = residual.array()*residual.array();
        rmse += squared_residuals;
    }

    //calculate the mean
    VectorXd mean = rmse/estimations.size();

    //calculate the squared root
    rmse = mean.array().sqrt();

    //return the result
    return rmse;
}
