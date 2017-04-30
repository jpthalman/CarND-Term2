/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[])
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
    for (int i = 0; i < num_particles; ++i)
        particles.push_back( Particle(dist_x(gen), dist_y(gen), dist_theta(gen)) );
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // instantiate RNG
    std::default_random_engine gen;
    std::normal_distribution<double> std_normal(0, 1);

    // predict the state of each particle at t+dt
    for (Particle &p : particles)
    {
        double  x_noise = std_pos[0] * std_normal(gen),
                y_noise = std_pos[1] * std_normal(gen),
                yaw_noise = std_pos[2] * std_normal(gen);

        if (yaw_rate > 1e-3)
        {
            p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        }
        else if (yaw_rate < 1e-3)
        {
            p.x += velocity * cos(p.theta) * delta_t;
            p.y += velocity * sin(p.theta) * delta_t;
            p.theta += yaw_rate * delta_t;
        }

        double dt2 = pow(delta_t, 2);

        // add noise
        p.x += 0.5 * dt2 * x_noise;
        p.y += 0.5 * dt2 * y_noise;
        p.theta += 0.5 * dt2 * yaw_noise;
    } // end particle updates

    return;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
