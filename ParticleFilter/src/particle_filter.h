/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

struct Particle
{
	int id;
	double x;
	double y;
	double theta;
	double weight;

	Particle() = default;
    Particle(double init_x, double init_y, double init_theta) :
            x(init_x), y(init_y), theta(init_theta), weight(1.0) {}
};



class ParticleFilter
{
	// Number of particles to draw
	int n_particles_;

	// Flag, if filter is initialized
	bool is_initialized_;
	
	// Vector of weights of all particles
	std::vector<double> weights_;
	
public:
	
	// Set of current particles
	std::vector<Particle> particles_;

	// Constructor
	// @param M Number of particles
	ParticleFilter(int init_n_particles = 500) :
			n_particles_(init_n_particles), is_initialized_(false) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
	
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
	void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	 *   standard deviation of bearing [rad]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations,
			Map map_landmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();
	
	/**
	 * write Writes particle positions to a file.
	 * @param filename File to write particle positions to.
	 * @param delimiter Separator for the values in each line.
	 */
	void write(std::string filename, std::string delimiter);
	
	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const { return is_initialized_; }
};



#endif /* PARTICLE_FILTER_H_ */
