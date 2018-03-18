#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(
    const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) 
{
    VectorXd RMSE(4);
    RMSE << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size

    if (estimations.size() == 0) 
    {
        cout << "Error: empty estimation list" << endl;
        return RMSE;
    }

    if (estimations.size() != ground_truth.size()) 
    {
        cout << "Error: wrong number of estimations" << endl;
        return RMSE;
    }

    // accumulate squared residuals
    for(size_t i=0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        RMSE += residual.cwiseProduct(residual);
    }

    // calculate the mean
    RMSE *= 1.0/estimations.size();

    //calculate the squared root
    RMSE = RMSE.array().sqrt();

    //return the result
    return RMSE;
}