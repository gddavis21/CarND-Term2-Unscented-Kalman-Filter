#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

static const double PI = acos(-1.0);
static const int STATE_DIM = 5;
static const int AUG_STATE_DIM = STATE_DIM + 2;
static const int NUM_SIGMA_PTS = 2*AUG_STATE_DIM + 1;

static const int RADAR_DIM = 3;
static const int LIDAR_DIM = 2;

namespace
{
    // ensure angle stays within [-PI,PI) range
    inline double NormalizeAngle(double angle)
    {
        return fmod(angle + PI, 2*PI) - PI;
    }

    inline VectorXd NormalizeState(const VectorXd &state_diff)
    {
        VectorXd n = state_diff;
        n(3) = NormalizeAngle(state_diff(3));  // normalize yaw angle
        return n;
    }

    inline VectorXd NormalizeRadar(const VectorXd &radar_diff)
    {
        VectorXd n = radar_diff;
        n(1) = NormalizeAngle(radar_diff(1));  // normalize heading angle
        return n;
    }
}

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() 
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // n_x_ = 5;  // state vector dimensions
    // n_aug_ = 7; // augmented state vector dimensions

    // // initial state vector
    // x_ = VectorXd(n_x_);

    // // initial covariance matrix
    // P_ = MatrixXd(n_x_, n_x_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */

    lambda_ = 3.0 - AUG_STATE_DIM;
    
    weights_ = VectorXd(NUM_SIGMA_PTS);
    weights_(0) = lambda_ / (lambda_ + AUG_STATE_DIM);
    for (int i=1; i < NUM_SIGMA_PTS; i++) {
        weights_(i) = 0.5 / (lambda_ + AUG_STATE_DIM);
    }

    proc_noise_covar_ = MatrixXd(2,2);
    proc_noise_covar_ << 
        std_a_*std_a_, 0, 
        0, std_yawdd_*std_yawdd_;

    radar_noise_covar_ = MatrixXd(3,3);
    radar_noise_covar_ <<
        std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

    laser_noise_covar_ = MatrixXd(2,2);
    laser_noise_covar_ <<
        std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
    VectorXd actual_meas = meas_package.raw_measurements_;

    // TODO: initialize on 1st measurement
    if (!is_initialized_) 
    {
        // extract initial position from lidar/radar measurement
        double x, y;

        switch (meas_package.sensor_type_) 
        {
            case MeasurementPackage::LASER:
                // Laser measurement already in cartesian coordinates
                x = actual_meas(0);
                y = actual_meas(1);
                break;

            case MeasurementPackage::RADAR:
            {
                // Convert radar measurement from polar to cartesian coordinates
                double range = actual_meas(0);
                double heading = actual_meas(1);
                x = range*cos(heading);
                y = range*sin(heading);
                break;
            }

            default:
                throw std::invalid_argument("unrecognized sensor type");
        }

        state_mean_ = VectorXd(STATE_DIM); // TODO
        state_mean_ << x, y, 1, 0, 0;

        state_covariance_ = MatrixXd::Identity(STATE_DIM,STATE_DIM); // TODO

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    // ignore Lidar measurement if Lidar processing disabled
    if (!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
        return;
    }

    // ignore Radar measurement if Radar processing disabled
    if (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        return;
    }

    double delta_t = (meas_package.timestamp_ - time_us_) * 1e-6;  // time delta in seconds
    time_us_ = meas_package.timestamp_;

    // predict state at current time
    UKF_Prediction pred_state = PredictState(delta_t);

    // update state with current measurement
    switch (meas_package.sensor_type_) 
    {
        case MeasurementPackage::LASER:
            UpdateLidar(pred_state, actual_meas);
            break;

        case MeasurementPackage::RADAR:
            UpdateRadar(pred_state, actual_meas);
            break;

        default:
            throw std::invalid_argument("unrecognized sensor type");
    }
}

VectorXd UKF::GetCurrentState() const
{
    return state_mean_;
}

// /**
//  * Predicts sigma points, the state, and the state covariance matrix.
//  * @param {double} delta_t the change in time (in seconds) between the last
//  * measurement and this one.
//  */
// void UKF::Prediction(double delta_t) 
// {
//     /**
//     TODO:

//     Complete this function! Estimate the object's location. Modify the state
//     vector, x_. Predict sigma points, the state, and the state covariance matrix.
//     */
// }

// /**
//  * Updates the state and the state covariance matrix using a laser measurement.
//  * @param {MeasurementPackage} meas_package
//  */
// void UKF::UpdateLidar(MeasurementPackage meas_package) 
// {
//     /**
//     TODO:

//     Complete this function! Use lidar data to update the belief about the object's
//     position. Modify the state vector, x_, and covariance, P_.

//     You'll also need to calculate the lidar NIS.
//     */
// }

// /**
//  * Updates the state and the state covariance matrix using a radar measurement.
//  * @param {MeasurementPackage} meas_package
//  */
// void UKF::UpdateRadar(MeasurementPackage meas_package) 
// {
//     /**
//     TODO:

//     Complete this function! Use radar data to update the belief about the object's
//     position. Modify the state vector, x_, and covariance, P_.

//     You'll also need to calculate the radar NIS.
//     */
// }

MatrixXd UKF::AugmentedSigmaPoints() const
{
    // augmented state mean
    VectorXd aug_state(AUG_STATE_DIM);
    aug_state << state_mean_, VectorXd::Zero(2);

    // augmented state covariance
    MatrixXd aug_covar(AUG_STATE_DIM, AUG_STATE_DIM);
    aug_covar <<
        state_covariance_, MatrixXd::Zero(STATE_DIM, 2),
        MatrixXd::Zero(2, STATE_DIM), proc_noise_covar_;

    MatrixXd root_covar = aug_covar.llt().matrixL();
    double f = sqrt(lambda_ + AUG_STATE_DIM);

    MatrixXd sigma_pts(AUG_STATE_DIM, NUM_SIGMA_PTS);
    sigma_pts.col(0) = aug_state;

    for (int i=0; i < AUG_STATE_DIM; i++) {
        sigma_pts.col(                i+1) = aug_state + f*root_covar.col(i);
        sigma_pts.col(AUG_STATE_DIM + i+1) = aug_state - f*root_covar.col(i);
    }

    return sigma_pts;
}

VectorXd UKF::StateTransition(const VectorXd &prior, double dt) 
{
    double x = prior(0);
    double y = prior(1);
    double s = prior(2);
    double yaw = prior(3);
    double yaw_rate = prior(4);
    double noise_lon = prior(5);
    double noise_yaw = prior(6);

    double x_pred = x;
    double y_pred = y;
    double s_pred = s;
    double yaw_pred = yaw;
    double yaw_rate_pred = yaw_rate;

    if (fabs(yaw_rate) < 0.001) {
        x_pred += s*cos(yaw)*dt;
        y_pred += s*sin(yaw)*dt;
    }
    else {
        x_pred += (s/yaw_rate)*(sin(yaw + yaw_rate*dt) - sin(yaw));
        y_pred += (s/yaw_rate)*(cos(yaw) - cos(yaw + yaw_rate*dt));
        yaw_pred += yaw_rate*dt;
    }

    x_pred += 0.5*dt*dt*cos(yaw)*noise_lon;
    y_pred += 0.5*dt*dt*sin(yaw)*noise_lon;
    s_pred += dt*noise_lon;
    yaw_pred += 0.5*dt*dt*noise_yaw;
    yaw_rate_pred += dt*noise_yaw;

    VectorXd pred(STATE_DIM);
    pred << x_pred, y_pred, s_pred, yaw_pred, yaw_rate_pred;
    return pred;
}

UKF::UKF_Prediction UKF::PredictState(double delta_t) const
{
    MatrixXd aug_sigma_pts = AugmentedSigmaPoints();
    
    UKF_Prediction pred_state;
    pred_state.sigma_pts = MatrixXd(STATE_DIM, NUM_SIGMA_PTS);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        pred_state.sigma_pts.col(i) = StateTransition(aug_sigma_pts.col(i), delta_t);
    }

    pred_state.mean = pred_state.sigma_pts * weights_;
    pred_state.covariance = MatrixXd::Zero(STATE_DIM, STATE_DIM);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        VectorXd dx = NormalizeState(pred_state.sigma_pts.col(i) - pred_state.mean);
        pred_state.covariance += (dx * dx.transpose()) * weights_(i);
    }

    return pred_state;
}

VectorXd UKF::StateToRadar(const VectorXd &state)
{
    double x = state(0);
    double y = state(1);
    double s = state(2);
    double yaw = state(3);

    double range = sqrt(x*x + y*y);
    // TODO: check for range ~ 0 (prevent divide-by-zero)
    double heading = atan2(y,x);
    double range_rate = (x*s*cos(yaw) + y*s*sin(yaw)) / range;

    VectorXd radar(RADAR_DIM);
    radar << range, heading, range_rate;
    return radar;
}

UKF::UKF_Prediction UKF::PredictRadarMeasurement(const UKF_Prediction &pred_state) const
{
    UKF_Prediction pred_meas;
    pred_meas.sigma_pts = MatrixXd(RADAR_DIM, NUM_SIGMA_PTS);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        pred_meas.sigma_pts.col(i) = StateToRadar(pred_state.sigma_pts.col(i));
    }

    pred_meas.mean = pred_meas.sigma_pts * weights_;
    pred_meas.covariance = radar_noise_covar_;

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        VectorXd dz = NormalizeRadar(pred_meas.sigma_pts.col(i) - pred_meas.mean);
        pred_meas.covariance += (dz * dz.transpose()) * weights_(i);
    }

    return pred_meas;
}

void UKF::UpdateRadar(const UKF_Prediction &pred_state, const VectorXd &actual_meas)
{
    // predict measurement from predicted state
    UKF_Prediction pred_meas = PredictRadarMeasurement(pred_state);

    // calculate state/measurement cross-correlation
    MatrixXd cross_corr = MatrixXd::Zero(STATE_DIM, RADAR_DIM);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        VectorXd dx = NormalizeState(pred_state.sigma_pts.col(i) - pred_state.mean);
        VectorXd dz = NormalizeRadar(pred_meas.sigma_pts.col(i) - pred_meas.mean);
        cross_corr += (dx * dz.transpose()) * weights_(i);
    }

    // calculate Kalman gain
    MatrixXd K = cross_corr * pred_meas.covariance.inverse();

    // update state mean and covariance
    VectorXd y = NormalizeRadar(actual_meas - pred_meas.mean);
    state_mean_ = pred_state.mean + K*y;
    state_covariance_ = pred_state.covariance - K*pred_meas.covariance*K.transpose();
}

VectorXd UKF::StateToLidar(const VectorXd &state)
{
    VectorXd lidar(LIDAR_DIM);
    lidar << state(0), state(1);
    return lidar;
}

UKF::UKF_Prediction UKF::PredictLidarMeasurement(const UKF_Prediction &pred_state) const
{
    UKF_Prediction pred_meas;
    pred_meas.sigma_pts = MatrixXd(LIDAR_DIM, NUM_SIGMA_PTS);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        pred_meas.sigma_pts.col(i) = StateToLidar(pred_state.sigma_pts.col(i));
    }

    pred_meas.mean = pred_meas.sigma_pts * weights_;
    pred_meas.covariance = laser_noise_covar_;

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        VectorXd dz = pred_meas.sigma_pts.col(i) - pred_meas.mean;
        pred_meas.covariance += (dz * dz.transpose()) * weights_(i);
    }

    return pred_meas;
}

void UKF::UpdateLidar(const UKF_Prediction &pred_state, const VectorXd &actual_meas)
{
    // predict measurement from predicted state
    UKF_Prediction pred_meas = PredictLidarMeasurement(pred_state);

    // calculate state/measurement cross-correlation
    MatrixXd cross_corr = MatrixXd::Zero(STATE_DIM, LIDAR_DIM);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        VectorXd dx = NormalizeState(pred_state.sigma_pts.col(i) - pred_state.mean);
        VectorXd dz = pred_meas.sigma_pts.col(i) - pred_meas.mean;
        cross_corr += (dx * dz.transpose()) * weights_(i);
    }

    // calculate Kalman gain
    MatrixXd K = cross_corr * pred_meas.covariance.inverse();

    // update state mean and covariance
    VectorXd y = actual_meas - pred_meas.mean;
    state_mean_ = pred_state.mean + K*y;
    state_covariance_ = pred_state.covariance - K*pred_meas.covariance*K.transpose();
}

// void UKF::UpdateState(
//     const UKF_Prediction &pred_state,
//     const UKF_Prediction &pred_meas,
//     const VectorXd &actual_meas)
// {
//     // calculate state/measurement cross-correlation
//     int n_x = pred_state.mean.rows();
//     int n_z = pred_meas.mean.rows();
//     int n_sig = pred_state.sigma_pts.cols();
//     MatrixXd cross_corr = MatrixXd::Zero(n_x, n_z);

//     for (int i=0; i < n_sig; i++) {
//         VectorXd dx = pred_state.sigma_pts.col(i) - pred_state.mean;
//         VectorXd dz = pred_meas.sigma_pts.col(i) - pred_meas.mean;
//         cross_corr += (dx * dz.transpose()) * weights_(i);
//     }

//     // calculate Kalman gain
//     MatrixXd K = cross_corr * pred_meas.covariance.inverse();

//     // update state mean and covariance
//     state_mean_ = pred_state.mean + K*(actual_meas - pred_meas.mean);
//     state_covariance_ = pred_state.covariance - K*pred_meas.covariance*K.transpose();
// }



// ****************************************************************************
//  UKF Lecture Exercises:

// void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) 
// {
//     //set state dimension
//     int n_x = 5;

//     //define spreading parameter
//     double lambda = 3 - n_x;

//     //set example state
//     VectorXd x = VectorXd(n_x);
//     x << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

//     //set example covariance matrix
//     MatrixXd P = MatrixXd(n_x, n_x);
//     P <<     
//          0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
//         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
//          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
//         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
//         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

//     //create sigma point matrix
//     MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

//     //calculate square root of P
//     MatrixXd A = P.llt().matrixL();

// /*******************************************************************************
//  * Student part begin
//  ******************************************************************************/

//     //your code goes here 

//     //calculate sigma points ...
//     //set sigma points as columns of matrix Xsig
//     MatrixXd B = A*sqrt(lambda + n_x);
//     Xsig.col(0) = x;
//     for (int i=0; i < n_x; i++)
//         Xsig.col(i+1) = (x + B.col(i));
//     for (int i=0; i < n_x; i++)
//         Xsig.col(n_x + i+1) = (x - B.col(i));

// /*******************************************************************************
//  * Student part end
//  ******************************************************************************/

//     //print result
//     //std::cout << "Xsig = " << std::endl << Xsig << std::endl;

//     //write result
//     *Xsig_out = Xsig;

// /* expected result:
//    Xsig =
//     5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
//       1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
//     2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
//     0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
//     0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
// */

// }

// void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) 
// {
//     //set state dimension
//     int n_x = 5;

//     //set augmented dimension
//     int n_aug = 7;

//     //Process noise standard deviation longitudinal acceleration in m/s^2
//     double std_a = 0.2;

//     //Process noise standard deviation yaw acceleration in rad/s^2
//     double std_yawdd = 0.2;

//     //define spreading parameter
//     double lambda = 3 - n_aug;

//     //set example state
//     VectorXd x = VectorXd(n_x);
//     x <<   5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

//     //create example covariance matrix
//     MatrixXd P = MatrixXd(n_x, n_x);
//     P <<     
//          0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
//         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
//          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
//         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
//         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

//     //create augmented mean vector
//     VectorXd x_aug = VectorXd(7);

//     //create augmented state covariance
//     MatrixXd P_aug = MatrixXd(7, 7);

//     //create sigma point matrix
//     MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

//     /*******************************************************************************
//      * Student part begin
//      ******************************************************************************/
    
//     //create augmented mean state
//     x_aug << x, VectorXd::Zero(2);

//     //create augmented covariance matrix
//     MatrixXd Q(2,2);
//     Q << std_a*std_a, 0, 0, std_yawdd*std_yawdd;
//     P_aug << P, MatrixXd::Zero(n_x, n_aug-n_x), MatrixXd::Zero(n_aug-n_x, n_x), Q;

//     //create square root matrix
//     MatrixXd A = P_aug.llt().matrixL();

//     //create augmented sigma points
//     MatrixXd B = A*sqrt(lambda + n_aug);
//     Xsig_aug.col(0) = x_aug;

//     for (int i=0; i < n_aug; i++) {
//         Xsig_aug.col(i+1) = x_aug + B.col(i);
//         Xsig_aug.col(i+1 + n_aug) = x_aug - B.col(i);
//     }
  
//     /*******************************************************************************
//      * Student part end
//      ******************************************************************************/

//     //print result
//     std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

//     //write result
//     *Xsig_out = Xsig_aug;

// /* expected result:
//    Xsig_aug =
//   5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
//     1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
//   2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
//   0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
//   0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
//        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
//        0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
// */

// }

// void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) 
// {
//     //set state dimension
//     int n_x = 5;

//     //set augmented dimension
//     int n_aug = 7;

//     int n_sig = 2*n_aug + 1;

//     //create example sigma point matrix
//     MatrixXd Xsig_aug = MatrixXd(n_aug, n_sig);
//     Xsig_aug <<
//         5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
//         1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
//         2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
//         0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
//         0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
//         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
//         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

//     //create matrix with predicted sigma points as columns
//     MatrixXd Xsig_pred = MatrixXd(n_x, n_sig);

//     double dt = 0.1; //time diff in sec

//     /*******************************************************************************
//      * Student part begin
//      ******************************************************************************/

//     //predict sigma points
//     for (int i=0; i < n_sig; i++) 
//     {
//         const VectorXd &sig = Xsig_aug.col(i);
//         double px = sig(0);
//         double py = sig(1);
//         double s = sig(2);
//         double yaw = sig(3);
//         double yaw_rate = sig(4);
//         double noise_lon = sig(5);
//         double noise_yaw = sig(6);

//         VectorXd pred(5);
//         pred << px, py, s, yaw, yaw_rate;

//         if (fabs(yaw_rate) < 0.001) {
//             pred(0) += s*cos(yaw)*dt;
//             pred(1) += s*sin(yaw)*dt;
//         }
//         else {
//             pred(0) += (s/yaw_rate)*(sin(yaw + yaw_rate*dt) - sin(yaw));
//             pred(1) += (s/yaw_rate)*(cos(yaw) - cos(yaw + yaw_rate*dt));
//             pred(3) += yaw_rate*dt;
//         }

//         pred(0) += 0.5*dt*dt*cos(yaw)*noise_lon;
//         pred(1) += 0.5*dt*dt*sin(yaw)*noise_lon;
//         pred(2) += dt*noise_lon;
//         pred(3) += 0.5*dt*dt*noise_yaw;
//         pred(4) += dt*noise_yaw;

//         Xsig_pred.col(i) = pred;
//     }
//     //avoid division by zero
//     //write predicted sigma points into right column
    

//     /*******************************************************************************
//      * Student part end
//      ******************************************************************************/

//     //print result
//     std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

//     //write result
//     *Xsig_out = Xsig_pred;

// }

// void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) 
// {
//     //set state dimension
//     int n_x = 5;

//     //set augmented dimension
//     int n_aug = 7;
//     int n_sig = 2*n_aug + 1;

//     //define spreading parameter
//     double lambda = 3 - n_aug;

//     //create example matrix with predicted sigma points
//     MatrixXd Xsig_pred = MatrixXd(n_x, n_sig);
//     Xsig_pred <<
//         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
//         1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
//         2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
//         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
//         0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

//     //create vector for weights
//     VectorXd weights = VectorXd(n_sig);

//     //create vector for predicted state
//     VectorXd x = VectorXd(n_x);

//     //create covariance matrix for prediction
//     MatrixXd P = MatrixXd::Zero(n_x, n_x);


//     /*******************************************************************************
//      * Student part begin
//      ******************************************************************************/

//     //set weights
//     weights(0) = lambda / (lambda + n_aug);
//     for (int i=1; i < n_sig; i++) {
//         weights(i) = 0.5 / (lambda + n_aug);
//     }

//     //predict state mean
//     x = Xsig_pred*weights;

//     //predict state covariance matrix
//     for (int i=0; i < n_sig; i++) {
//         VectorXd dx = Xsig_pred.col(i) - x;
//         // should normalize dx(3) (yaw angle difference)
//         P += (dx * dx.transpose()) * weights(i);
//     }

//     /*******************************************************************************
//      * Student part end
//      ******************************************************************************/

//     //print result
//     std::cout << "Predicted state" << std::endl;
//     std::cout << x << std::endl;
//     std::cout << "Predicted covariance matrix" << std::endl;
//     std::cout << P << std::endl;

//     //write result
//     *x_out = x;
//     *P_out = P;
// }

// void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) 
// {
//     //set state dimension
//     int n_x = 5;

//     //set augmented dimension
//     int n_aug = 7;

//     //set measurement dimension, radar can measure r, phi, and r_dot
//     int n_z = 3;

//     //define spreading parameter
//     double lambda = 3 - n_aug;

//     //set vector for weights
//     VectorXd weights = VectorXd(2*n_aug+1);
//     double weight_0 = lambda/(lambda+n_aug);
//     weights(0) = weight_0;
//     for (int i=1; i<2*n_aug+1; i++) {  
//         double weight = 0.5/(n_aug+lambda);
//         weights(i) = weight;
//     }

//     //radar measurement noise standard deviation radius in m
//     double std_radr = 0.3;

//     //radar measurement noise standard deviation angle in rad
//     double std_radphi = 0.0175;

//     //radar measurement noise standard deviation radius change in m/s
//     double std_radrd = 0.1;

//     //create example matrix with predicted sigma points
//     MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
//     Xsig_pred <<
//         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
//         1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
//         2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
//         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
//         0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

//     //create matrix for sigma points in measurement space
//     MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

//     //mean predicted measurement
//     VectorXd z_pred = VectorXd(n_z);
    
//     //measurement covariance matrix S
//     MatrixXd S = MatrixXd(n_z,n_z);

//     /*******************************************************************************
//      * Student part begin
//      ******************************************************************************/

//     //transform sigma points into measurement space
//     int n_sig = 2*n_aug + 1;

//     for (int i=0; i < n_sig; i++) 
//     {
//         double px = Xsig_pred(0,i);
//         double py = Xsig_pred(1,i);
//         double s = Xsig_pred(2,i);
//         double yaw = Xsig_pred(3,i);
//         double yaw_rate = Xsig_pred(4,i);

//         double range = sqrt(px*px + py*py);
//         double heading = atan2(py,px);
//         double range_rate = (px*s*cos(yaw) + py*s*sin(yaw)) / range;

//         Zsig.col(i) << range, heading, range_rate;
//     }
//     //calculate mean predicted measurement
//     z_pred = Zsig * weights;

//     //calculate innovation covariance matrix S
//     S.fill(0.0);
//     S(0,0) = std_radr*std_radr;
//     S(1,1) = std_radphi*std_radphi;
//     S(2,2) = std_radrd*std_radrd;

//     for (int i=0; i < n_sig; i++) 
//     {
//         VectorXd dz = Zsig.col(i) - z_pred;
//         S += (dz * dz.transpose()) * weights(i);
//     }

    
//     /*******************************************************************************
//      * Student part end
//      ******************************************************************************/

//     //print result
//     std::cout << "z_pred: " << std::endl << z_pred << std::endl;
//     std::cout << "S: " << std::endl << S << std::endl;

//     //write result
//     *z_out = z_pred;
//     *S_out = S;
// }

// void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out) 
// {
//     //set state dimension
//     int n_x = 5;

//     //set augmented dimension
//     int n_aug = 7;

//     //set measurement dimension, radar can measure r, phi, and r_dot
//     int n_z = 3;

//     //define spreading parameter
//     double lambda = 3 - n_aug;

//     //set vector for weights
//     VectorXd weights = VectorXd(2*n_aug+1);
//     double weight_0 = lambda/(lambda+n_aug);
//     weights(0) = weight_0;
//     for (int i=1; i<2*n_aug+1; i++) {  
//         double weight = 0.5/(n_aug+lambda);
//         weights(i) = weight;
//     }

//     //create example matrix with predicted sigma points in state space
//     MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
//     Xsig_pred <<
//         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
//         1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
//         2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
//         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
//         0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

//     //create example vector for predicted state mean
//     VectorXd x = VectorXd(n_x);
//     x <<  5.93637, 1.49035, 2.20528, 0.536853, 0.353577;

//     //create example matrix for predicted state covariance
//     MatrixXd P = MatrixXd(n_x,n_x);
//     P <<
//         0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
//         -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
//         0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
//         -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
//         -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

//     //create example matrix with sigma points in measurement space
//     MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
//     Zsig <<
//         6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
//         0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
//         2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

//     //create example vector for mean predicted measurement
//     VectorXd z_pred = VectorXd(n_z);
//     z_pred << 6.12155, 0.245993, 2.10313;

//     //create example matrix for predicted measurement covariance
//     MatrixXd S = MatrixXd(n_z,n_z);
//     S <<
//         0.0946171, -0.000139448,   0.00407016,
//         -0.000139448,  0.000617548, -0.000770652,
//         0.00407016, -0.000770652,    0.0180917;

//     //create example vector for incoming radar measurement
//     VectorXd z = VectorXd(n_z);
//     z <<  
//         5.9214,   //rho in m
//         0.2187,   //phi in rad
//         2.0062;   //rho_dot in m/s

//     //create matrix for cross correlation Tc
//     MatrixXd Tc = MatrixXd(n_x, n_z);

//     /*******************************************************************************
//      * Student part begin
//      ******************************************************************************/
//     int n_sig = 2*n_aug + 1;

//     //calculate cross correlation matrix
//     Tc.fill(0.0);
//     for (int i=0; i < n_sig; i++) {
//         VectorXd dx = Xsig_pred.col(i) - x;
//         VectorXd dz = Zsig.col(i) - z_pred;
//         Tc += (dx * dz.transpose()) * weights(i);
//     }

//     //calculate Kalman gain K;
//     MatrixXd K = Tc * S.inverse();

//     //update state mean and covariance matrix
//     x += K * (z - z_pred);
//     P -= K * S * K.transpose();


//     /*******************************************************************************
//      * Student part end
//      ******************************************************************************/

//     //print result
//     std::cout << "Updated state x: " << std::endl << x << std::endl;
//     std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

//     //write result
//     *x_out = x;
//     *P_out = P;
// }

