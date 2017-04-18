//
// Created by japata on 4/17/17.
//

#ifndef UNSCENTEDKALMANFILTER_BASEEKF_H
#define UNSCENTEDKALMANFILTER_BASEEKF_H

#include <vector>
#include "Eigen/Dense"
#include "sensor_data_packet.h"


class BaseEKF {
public:
    BaseEKF(int n_states, std::vector<double> noise_stdevs, double lambda);

    Eigen::VectorXd GetCurrentState() const { return x_; }
    Eigen::MatrixXd GetCurrentCovariance() const { return P_; }
    void ProcessMeasurement(SensorDataPacket &data);

protected:
    virtual Eigen::MatrixXd PredictSigmaPoints(Eigen::MatrixXd &sigma_pts, double delta_t);
    virtual Eigen::MatrixXd SigmaPointsToMeasurementSpace(Eigen::MatrixXd &sigma_pts,
                                                          SensorDataPacket::SensorType sensor_type);

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

private:
    void GenerateSigmaPoints(Eigen::VectorXd &x, Eigen::MatrixXd &P);
    void GetMeanAndCovariance();

    int n_states_;
    int n_aug_states_;
    int lambda_;

    bool is_initialized_;
    long long prev_timestamp_;
};


#endif //UNSCENTEDKALMANFILTER_BASEEKF_H
