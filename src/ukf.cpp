#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;

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
}

UKF::UKF(
    RadarUncertainty radar_noise,
    LidarUncertainty lidar_noise,
    bool use_radar,
    bool use_lidar) 
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = use_lidar;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = use_radar;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.5;  // initial guess

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.25*PI;  // initial guess

    proc_noise_covar_ = MatrixXd(2,2);
    proc_noise_covar_ << 
        std_a_*std_a_, 0, 
        0, std_yawdd_*std_yawdd_;

    // Laser measurement noise standard deviation
    std_laspx_ = lidar_noise.std_x;  // in m
    std_laspy_ = lidar_noise.std_y;  // in m

    laser_noise_covar_ = MatrixXd(2,2);
    laser_noise_covar_ <<
        std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

    // Radar measurement noise standard deviation
    std_radr_   = radar_noise.std_range;      // in m
    std_radphi_ = radar_noise.std_heading;    // in radians
    std_radrd_  = radar_noise.std_range_rate; // in m/s

    radar_noise_covar_ = MatrixXd(3,3);
    radar_noise_covar_ <<
        std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

    // sigma point spread parameter
    lambda_ = 3.0 - AUG_STATE_DIM;
    
    // sigma point weights
    weights_ = VectorXd(NUM_SIGMA_PTS);
    weights_(0) = lambda_ / (lambda_ + AUG_STATE_DIM);
    for (int i=1; i < NUM_SIGMA_PTS; i++) {
        weights_(i) = 0.5 / (lambda_ + AUG_STATE_DIM);
    }

    NIS_radar_ = NIS_lidar_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
    // initialize on 1st measurement
    if (!is_initialized_) 
    {
        // extract initial position from lidar/radar measurement
        VectorXd actual_meas = meas_package.raw_measurements_;
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

        const double INITIAL_SPEED = 6;
        const double INITIAL_VARIANCE = 0.05;

        state_mean_ = VectorXd(STATE_DIM);
        state_mean_ << x, y, INITIAL_SPEED, 0, 0;

        state_covariance_ = MatrixXd::Identity(STATE_DIM,STATE_DIM) * INITIAL_VARIANCE;

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

    // time delta in seconds
    double delta_t = (meas_package.timestamp_ - time_us_) * 1e-6;

    // predict state at current time
    UKF_Prediction pred_state = PredictState(delta_t);

    // update state with current measurement
    switch (meas_package.sensor_type_) 
    {
        case MeasurementPackage::LASER:
            UpdateLidar(pred_state, meas_package.raw_measurements_);
            break;

        case MeasurementPackage::RADAR:
            UpdateRadar(pred_state, meas_package.raw_measurements_);
            break;

        default:
            throw std::invalid_argument("unrecognized sensor type");
    }

    time_us_ = meas_package.timestamp_;
}

Vector2d UKF::GetCurrentPosition() const 
{
    return Vector2d(state_mean_(0), state_mean_(1));
}

Vector2d UKF::GetCurrentVelocity() const 
{
    double speed = state_mean_(2);
    double yaw = state_mean_(3);
    return Vector2d(speed*cos(yaw), speed*sin(yaw));
}

// ensure -PI <= state.yaw < PI
VectorXd UKF::NormalizeState(const VectorXd &state)
{
    VectorXd n = state;
    n(3) = NormalizeAngle(state(3));  // normalize yaw angle
    return n;
}

// ensure -PI <= radar.heading < PI
VectorXd UKF::NormalizeRadar(const VectorXd &radar)
{
    VectorXd n = radar;
    n(1) = NormalizeAngle(radar(1));  // normalize heading angle
    return n;
}

// compute UKF augmented sigma points
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

// predict state mean, given prior (augmented) state mean and time offset
VectorXd UKF::StateTransition(const VectorXd &prior, double delta_t) 
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
        x_pred += s*cos(yaw)*delta_t;
        y_pred += s*sin(yaw)*delta_t;
    }
    else {
        x_pred += (s/yaw_rate)*(sin(yaw + yaw_rate*delta_t) - sin(yaw));
        y_pred += (s/yaw_rate)*(cos(yaw) - cos(yaw + yaw_rate*delta_t));
        yaw_pred += yaw_rate*delta_t;
    }

    x_pred += 0.5*delta_t*delta_t*cos(yaw)*noise_lon;
    y_pred += 0.5*delta_t*delta_t*sin(yaw)*noise_lon;
    s_pred += delta_t*noise_lon;
    yaw_pred += 0.5*delta_t*delta_t*noise_yaw;
    yaw_rate_pred += delta_t*noise_yaw;

    VectorXd pred(STATE_DIM);
    pred << x_pred, y_pred, s_pred, yaw_pred, yaw_rate_pred;
    return pred;
}

// predict sigma points, state mean & state covariance
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

// compute radar measurement from given state vector
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

// convert predicted state to predicted radar measurement
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

// update state belief, given predicted state and actual radar measurement
void UKF::UpdateRadar(const UKF_Prediction &pred_state, const VectorXd &actual_meas)
{
    // predict measurement from predicted state
    UKF_Prediction pred_meas = PredictRadarMeasurement(pred_state);

    // calculate state/measurement cross-correlation
    MatrixXd T = MatrixXd::Zero(STATE_DIM, RADAR_DIM);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        VectorXd dx = NormalizeState(pred_state.sigma_pts.col(i) - pred_state.mean);
        VectorXd dz = NormalizeRadar(pred_meas.sigma_pts.col(i) - pred_meas.mean);
        T += (dx * dz.transpose()) * weights_(i);
    }

    // calculate Kalman gain
    const MatrixXd &S = pred_meas.covariance;
    MatrixXd S_inv = S.inverse();
    MatrixXd K = T*S_inv;

    // update state mean and covariance
    VectorXd y = NormalizeRadar(actual_meas - pred_meas.mean);
    state_mean_ = pred_state.mean + K*y;
    state_covariance_ = pred_state.covariance - K*S*K.transpose();

    // normalized innovation squared metric
    NIS_radar_ = y.transpose() * S_inv * y;
}

// compute lidar measurement from given state vector
VectorXd UKF::StateToLidar(const VectorXd &state)
{
    VectorXd lidar(LIDAR_DIM);
    lidar << state(0), state(1);
    return lidar;
}

// convert predicted state to predicted lidar measurement
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

// update state belief, given predicted state and actual lidar measurement
void UKF::UpdateLidar(const UKF_Prediction &pred_state, const VectorXd &actual_meas)
{
    // predict measurement from predicted state
    UKF_Prediction pred_meas = PredictLidarMeasurement(pred_state);

    // calculate state/measurement cross-correlation
    MatrixXd T = MatrixXd::Zero(STATE_DIM, LIDAR_DIM);

    for (int i=0; i < NUM_SIGMA_PTS; i++) {
        VectorXd dx = NormalizeState(pred_state.sigma_pts.col(i) - pred_state.mean);
        VectorXd dz = pred_meas.sigma_pts.col(i) - pred_meas.mean;
        T += (dx * dz.transpose()) * weights_(i);
    }

    // calculate Kalman gain
    const MatrixXd &S = pred_meas.covariance;
    MatrixXd S_inv = S.inverse();
    MatrixXd K = T*S_inv;

    // update state mean and covariance
    VectorXd y = actual_meas - pred_meas.mean;
    state_mean_ = pred_state.mean + K*y;
    state_covariance_ = pred_state.covariance - K*S*K.transpose();

    // normalized innovation squared metric
    NIS_lidar_ = y.transpose() * S_inv * y;
}
