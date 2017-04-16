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
    UnscentedKalmanFilter(int init_n_states, std::vector<double> noise_vector) :
            n_states_(init_n_states),
            x_(Eigen::VectorXd(init_n_states)),
            P_(Eigen::MatrixXd::Identity(init_n_states, init_n_states)),
            sigma_points_(Eigen::MatrixXd(init_n_states, 2 * init_n_states + 1))
    {
        // Initialize noise covariances of the state covariance.
        // For example, if we have 3 states and one of them is noise, then P_ will look like:
        //
        // 1  0  0
        // 0  1  0
        // 0  0  noise_vector[0]^2
        for (int idx = init_n_states - noise_vector.size(); idx < init_n_states; ++idx) {
            P_(idx, idx) = pow(noise_vector[idx], 2);
        }
    }

    virtual ~UnscentedKalmanFilter();

    Eigen::VectorXd GetCurrentState() const { return x_.head(5); }
    Eigen::MatrixXd GetCurrentCovariance() const { return P_.block<5,5>(0, 0); }

    void ProcessMeasurement(SensorDataPacket &data);

private:
    void GenerateSigmaPoints(Eigen::VectorXd x, Eigen::MatrixXd P, int lambda);

    int n_states_;
    bool is_initialized_ = false;
    long long prev_timestamp_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance
    Eigen::MatrixXd P_;

    // sigma points
    Eigen::MatrixXd sigma_points_;
};


#endif //UNSCENTEDKALMANFILTER_UNSCENTED_KALMAN_FILTER_H
