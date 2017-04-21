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
#include <limits>

#include "particle_filter.h"

using namespace std; 

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	// x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Define the number of particles
	num_particles = 1000; 

	// Resize wights and particle of the filter regarding the number of particles
	weights.resize(num_particles);
    particles.resize(num_particles);

	// Standard deviation for x, y and psi 
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2]; 

	// Normal distribution for x, y and psi
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	default_random_engine gen;

	for(int i = 0; i < num_particles; ++i){
		// Create a particle and set its value
		Particle p; 
		p.id = i; 
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen); 
		p.weight = 1; 	

		// Add this newly-created particle to the particle filter
		particles[i] = p;
		weights[i] = p.weight; 
	}

	is_initialized = true; 

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Standard deviation for x, y and psi 
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	default_random_engine gen;
	for(int i = 0; i < num_particles ; ++i){
		// Get the current particle 
		Particle *p = &particles[i]; // Pointer because we'll update it. 

		// Compute its updated  position
		double updated_x = p->x + velocity / yaw_rate * (sin(p->theta + yaw_rate * delta_t) - sin(p->theta));
		double updated_y = p->y + velocity / yaw_rate * (cos(p->theta) - cos(p->theta + yaw_rate * delta_t));
		double updated_theta =  p->theta + yaw_rate * delta_t;

		// Add Gaussian noise
		normal_distribution<double> dist_x(updated_x, std_x);
        normal_distribution<double> dist_y(updated_y, std_y);
        normal_distribution<double> dist_theta(updated_theta, std_theta);

		// Update the current particle
		p->x = dist_x(gen);
		p->y = dist_y(gen);
		p->theta = dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// For each predicted landmark, find the closest measurement and assign it to the landmark
	for(auto prediction : predicted){ // Iterate over predicted landmarks
		double smallest_dist = std::numeric_limits<double>::max(); // https://stackoverflow.com/questions/409348/iteration-over-stdvector-unsigned-vs-signed-index-variable
		for(auto observation : observations){
			// Get the distance between observations and landmark
			double dist_pred_obs = dist(observation.x, observation.y, prediction.x, prediction.y);
			if(dist_pred_obs < smallest_dist){
				observation.id = prediction.id; 
			}
			smallest_dist = dist_pred_obs; 
		}
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
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

	// Extract value
	double sigma_x = std_landmark[0];
	double sigma_y = std_landmark[1]; 

	// For each particle, convert observations from Vehicle to maps system
	for(int i = 0; i < num_particles ; i++){
		Particle *p = &particles[i];
		
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	// http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;

	// Random integers on the interval [0, n), where the probability of each individual integer i is defined as the weight of the ith integer divided by the sum of all n weights.
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resampled_particles;

	for (int i = 0; i < num_particles; i++){
		resampled_particles.push_back(particles[distribution(gen)]);
	}

	particles = resampled_particles;

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
