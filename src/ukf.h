#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

//   ///* State dimension
//   int n_x_;

//   ///* Augmented state dimension
//   int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

//   /**
//    * Prediction Predicts sigma points, the state, and the state covariance
//    * matrix
//    * @param delta_t Time between k and k+1 in s
//    */
//   void Prediction(double delta_t);

//   /**
//    * Updates the state and the state covariance matrix using a laser measurement
//    * @param meas_package The measurement at k+1
//    */
//   void UpdateLidar(MeasurementPackage meas_package);

//   /**
//    * Updates the state and the state covariance matrix using a radar measurement
//    * @param meas_package The measurement at k+1
//    */
//   void UpdateRadar(MeasurementPackage meas_package);

    Eigen::VectorXd GetCurrentState() const;

private:
    struct UKF_Prediction 
    {
        Eigen::MatrixXd sigma_pts;
        Eigen::VectorXd mean;
        Eigen::MatrixXd covariance;
    };

    Eigen::MatrixXd AugmentedSigmaPoints() const;
    UKF_Prediction PredictState(double delta_t) const;
    UKF_Prediction PredictRadarMeasurement(const UKF_Prediction &pred_state) const;
    UKF_Prediction PredictLidarMeasurement(const UKF_Prediction &pred_state) const;
    void UpdateRadar(const UKF_Prediction &pred_state, const VectorXd &actual_meas);
    void UpdateLidar(const UKF_Prediction &pred_state, const VectorXd &actual_meas);

    static Eigen::VectorXd StateTransition(const Eigen::VectorXd &prior, double delta_t);
    static Eigen::VectorXd StateToRadar(const Eigen::VectorXd &state);
    static Eigen::VectorXd StateToLidar(const Eigen::VectorXd &state);

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd state_mean_;

    // state covariance matrix
    Eigen::MatrixXd state_covariance_;

    // process noise covariance matrix
    Eigen::MatrixXd proc_noise_covar_;

    // radar/lidar measurement noise covariance matrices
    Eigen::MatrixXd radar_noise_covar_, laser_noise_covar_;
};

#endif /* UKF_H */
