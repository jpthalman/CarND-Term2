//
// Created by jacob on 4/18/2017.
//

#ifndef UNSCENTEDKALMANFILTER_RADARLIDARUKF_H
#define UNSCENTEDKALMANFILTER_RADARLIDARUKF_H


#include "BaseUKF.h"

/**
 * Extend BaseUKF to the case of a dynamic system with Radar and Lidar measurements.
 *
 * @param radar_noise: A vector which stores the noise STDDEV for each radar state.
 * @param lidar_noise: A vector which stores the noise STDDEV for each lidar state.
 * @param process_noise: A vector which stores the STDDEV of the process noises.
 * */
class RadarLidarUKF : BaseUKF
{
public:
    /**
     * Constructor and destructor.
     * */
    RadarLidarUKF(
            std::vector<float> radar_noise,
            std::vector<float> lidar_noise,
            std::vector<float> process_noise,
            double lambda);

    ~RadarLidarUKF() {};

    /**
     * Expose the base class member functions.
     * */
    using BaseUKF::GetCurrentState;
    using BaseUKF::GetCurrentCovariance;
    using BaseUKF::ProcessMeasurement;

private:
    /**
     * Implement BaseUKF's functions for this use case.
     * */
    Eigen::VectorXd InitializeState(
            const SensorDataPacket &data);

    Eigen::MatrixXd PredictSigmaPoints(
            const Eigen::MatrixXd &sigma_pts,
            const double delta_t);

    Eigen::MatrixXd SigmaPointsToMeasurementSpace(
            const Eigen::MatrixXd &sigma_pts,
            const Eigen::VectorXd &weights,
            const SensorDataPacket::SensorType sensor_type);

    void ProcessSpaceMeanAndCovariance(
            const Eigen::MatrixXd &sigma_pts,
            Eigen::VectorXd &mean,
            Eigen::MatrixXd &cov);

    void MeasurementSpaceMeanAndCovariance(
            const Eigen::MatrixXd &sigma_pts,
            const SensorDataPacket::SensorType &sensor_type,
            Eigen::VectorXd &mean,
            Eigen::MatrixXd &cov);

    Eigen::VectorXd StateSpaceToCartesian(const Eigen::VectorXd &x);

    Eigen::VectorXd NormalizeStateVector(const Eigen::VectorXd &x);

    Eigen::VectorXd NormalizeMeasurementVector(
            const Eigen::VectorXd &z,
            SensorDataPacket::SensorType sensor_type);

    // noise covariance matrices for the radar and lidar
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd R_lidar_;
};


#endif //UNSCENTEDKALMANFILTER_RADARLIDARUKF_H
