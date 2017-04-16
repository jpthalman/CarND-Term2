//
// Created by jaket on 4/13/2017.
//

#ifndef UNSCENTEDKALMANFILTER_UNSCENTED_KALMAN_FILTER_H
#define UNSCENTEDKALMANFILTER_UNSCENTED_KALMAN_FILTER_H

#include <vector>
#include "sensor_data_packet.h"
#include "Eigen/Dense"


class UnscentedKalmanFilter {
public:
    UnscentedKalmanFilter(int init_n_states = 5, float std_acc = 0.2, float std_yawdd = 0.2);

    virtual ~UnscentedKalmanFilter();

    Eigen::VectorXd GetCurrentState() const { return x_.head(5); }
    Eigen::MatrixXd GetCurrentCovariance() const { return P_.block<5,5>(0, 0); }

    void ProcessMeasurement(SensorDataPacket &data);

private:
    void GenerateSigmaPoints(Eigen::VectorXd &x, Eigen::MatrixXd &P, int lambda);
    void PredictSigmaPoints(Eigen::MatrixXd &sigma_pts, double delta_t);
    void PredictMeanAndCovariance(Eigen::MatrixXd &sigma_pts);

    int n_states_;
    int n_aug_states_;
    int lambda_;

    bool is_initialized_ = false;
    long long prev_timestamp_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance
    Eigen::MatrixXd P_;

    // process noise covariance
    Eigen::MatrixXd Q_;

    // sigma points
    Eigen::MatrixXd sigma_points_;

    // weights for prediction step
    Eigen::VectorXd weights_;
};


#endif //UNSCENTEDKALMANFILTER_UNSCENTED_KALMAN_FILTER_H
