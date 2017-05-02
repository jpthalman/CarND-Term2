/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#include <random>
#include <algorithm>
#include <iostream>

#include "particle_filter.h"

void ParticleFilter::init(
        double x,
        double y,
        double theta,
        double std[])
{
    std::default_random_engine gen;

    // shatter array for readability
    double  std_x = std[0],
            std_y = std[1],
            std_theta = std[2];

    std::normal_distribution<double>
            dist_x(x, std_x),
            dist_y(y, std_y),
            dist_theta(theta, std_theta);

    // num_particles is set with the constructor for this class, with a default of 1000.
    for (int i = 0; i < n_particles_; ++i)
        particles_.push_back( Particle(dist_x(gen), dist_y(gen), dist_theta(gen)) );

    is_initialized_ = true;
}

void ParticleFilter::prediction(
        double delta_t,
        double std_pos[],
        double velocity,
        double yaw_rate)
{
    // instantiate RNG
    std::default_random_engine gen;
    std::normal_distribution<double> std_normal(0, 1);

    // predict the state of each particle at t+dt
    for (Particle &p : particles_)
    {
        // don't divide by zero
        if (fabs(yaw_rate) > 1e-3)
        {
            p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        } else {
            p.x += velocity * cos(p.theta) * delta_t;
            p.y += velocity * sin(p.theta) * delta_t;
            p.theta += yaw_rate * delta_t;
        }

        // add noise
        double  dt2 = pow(delta_t, 2),
                x_noise = std_pos[0] * std_normal(gen),
                y_noise = std_pos[1] * std_normal(gen),
                yaw_noise = std_pos[2] * std_normal(gen);

        p.x += 0.5 * dt2 * x_noise;
        p.y += 0.5 * dt2 * y_noise;
        p.theta += 0.5 * dt2 * yaw_noise;
    } // end particle updates

    return;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations)
{
    // for each observed landmark
    for (LandmarkObs &obs : observations)
    {
        double min_distance = 1e6;

        // for each predicted landmark location
        for (int i = 0; i < predicted.size(); ++i)
        {
            // only associate predictions with one landmark
            if (predicted[i].id == 0)
            {
                double dist = euclidean_distance(obs.x, obs.y, predicted[i].x, predicted[i].y);

                if (dist < min_distance) {
                    obs.id = i;
                    min_distance = dist;
                }
            }
        } // end prediction loop
    } // end landmark loop

    return;
}

void ParticleFilter::updateWeights(
        double sensor_range,
        double std_landmark[],
		std::vector<LandmarkObs> observations,
        Map map_landmarks)
{
    // clear the weights so they can be updated
    weights_.clear();

    const double    std_x = std_landmark[0],
                    std_y = std_landmark[1],
                    c = 0.5 / (M_PI * std_x * std_y); // constant used to calculate multivariate normal dist.

    // for each particle
    for (Particle &p : particles_)
    {
        std::vector<LandmarkObs> observations_in_map_coords;
        std::vector<LandmarkObs> landmarks_in_sensor_range;

        // transform observations into map coordinates
        for (const LandmarkObs &obs : observations)
        {
            LandmarkObs transformed_obs;

            transformed_obs.id = 0;
            transformed_obs.x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
            transformed_obs.y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);

            observations_in_map_coords.push_back(transformed_obs);
        }

        // find landmarks in the map which are within sensor_range
        for (Map::single_landmark_s &lm : map_landmarks.landmark_list)
        {
            if (euclidean_distance(p.x, p.y, lm.x_f, lm.y_f) <= sensor_range)
                landmarks_in_sensor_range.push_back( LandmarkObs(lm) );
        }

        // associate in-range landmarks with transformed sensor readings
        dataAssociation(observations_in_map_coords, landmarks_in_sensor_range);

        // calculate the weight of the particle
        double prob = 1.0;

        for (const LandmarkObs &lm : landmarks_in_sensor_range)
        {
            const LandmarkObs &closest_obs = observations_in_map_coords[lm.id];

            double x_diff = pow((closest_obs.x - lm.x) / std_x, 2);
            double y_diff = pow((closest_obs.y - lm.y) / std_y, 2);

            prob *= c * exp(-0.5 * (x_diff + y_diff));
        }

        // Don't let a particle have zero probability of being chosen
        if (prob < 1e-4)
            prob = 1e-4;

        // store the probability of this particle being real in the weight member and the weights_ vector.
        p.weight = prob;
        weights_.push_back(prob);
    }
}

void ParticleFilter::resample()
{
    // Create random index generator where the probability of each particle index to be selected
    // is equivalent to its weight.
    std::default_random_engine gen;
    std::discrete_distribution<int> d(weights_.begin(), weights_.end());

    // temporary list to store re-sampled particles
    std::vector<Particle> resampled_particles;

    // generate a random index and append the corresponding particle to the re-sampling list
    for (int i = 0; i < n_particles_; ++i)
        resampled_particles.push_back( particles_[d(gen)] );

    // replace the old particles with the re-sampled particles
    particles_ = resampled_particles;
}

void ParticleFilter::write(std::string filename, std::string delimiter = ",")
{
	std::ofstream dataFile(filename, std::ios::app);

	for (int i = 0; i < n_particles_; ++i)
		dataFile << particles_[i].x << delimiter
                 << particles_[i].y << delimiter
                 << particles_[i].theta << std::endl;

	dataFile.close();
}
