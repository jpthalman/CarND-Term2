//
// Created by japata on 4/17/17.
//

#ifndef UNSCENTEDKALMANFILTER_BASEUKF_H
#define UNSCENTEDKALMANFILTER_BASEUKF_H

#include <vector>
#include "Eigen/Dense"
#include "sensor_data_packet.h"


class BaseUKF {
public:
    BaseUKF(int n_states, std::vector<float> noise_stdevs, double lambda);
    virtual ~BaseUKF() {}

    const Eigen::VectorXd &GetCurrentState() const { return x_; }
    const Eigen::MatrixXd &GetCurrentCovariance() const { return P_; }
    void ProcessMeasurement(SensorDataPacket &data);

protected:
    virtual Eigen::VectorXd InitializeState(
            const SensorDataPacket &data);

    virtual Eigen::MatrixXd PredictSigmaPoints(
            const Eigen::MatrixXd &sigma_pts,
            const double delta_t);

    virtual Eigen::MatrixXd SigmaPointsToMeasurementSpace(
            const Eigen::MatrixXd &sigma_pts,
            const Eigen::VectorXd &weights,
            const SensorDataPacket::SensorType sensor_type);

    virtual void ProcessSpaceMeanAndCovariance(
            const Eigen::MatrixXd &sigma_pts,
            Eigen::VectorXd &mean,
            Eigen::MatrixXd &cov);

    virtual void MeasurementSpaceMeanAndCovariance(
            const Eigen::MatrixXd &sigma_pts,
            const SensorDataPacket::SensorType &sensor_type,
            Eigen::VectorXd &mean,
            Eigen::MatrixXd &cov);

    virtual Eigen::VectorXd StateSpaceToCartesian(
            const Eigen::VectorXd &x);

    const int n_states_;
    const int n_aug_states_;
    bool is_initialized_;
    long long prev_timestamp_;

    // weights for prediction step
    Eigen::VectorXd weights_;

private:
    void GenerateSigmaPoints(const Eigen::VectorXd &x, const Eigen::MatrixXd &P);
    void CalculateKalmanGain();

    int lambda_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance
    Eigen::MatrixXd P_;

    // process noise covariance
    Eigen::MatrixXd Q_;

    // sigma points
    Eigen::MatrixXd X_sigma_points_;

    // measurement space sigma points
    Eigen::MatrixXd Z_sigma_points_;

    // measurement space mean
    Eigen::VectorXd z_;

    // measurement space covariance
    Eigen::MatrixXd S_;

    // kalman gain
    Eigen::MatrixXd K_;
};


#endif //UNSCENTEDKALMANFILTER_BASEUKF_H
