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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// Initialize Number of Particles
// Initialize position for each particle
// Initialize weight for each particle
// Initialize weight array
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set number of particles
	num_particles = 100;

	// Create random generator
	// Create a distribution to represent noise of GPS
	std::default_random_engine gen;
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
	std::normal_distribution<double> dist_x(x, std_x);
	std::normal_distribution<double> dist_y(y, std_y);
	std::normal_distribution<double> dist_theta(theta, std_theta);

	// Iinitialize each particle to a random coord centered on our gps.
	for (int i = 0; i < num_particles; i++){
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);

		p.weight = 1;
		//p.associations = {1, 2, 3};
		//p.sense_x = {1, 1, 1};
		//p.sense_y = {1,1,1};

		particles.push_back(p);
		weights.push_back(1);
	}

	is_initialized = true;
}


// Predict where particle should be, based off of control inputs (or odemetrics, not specified).
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Create a random generator
	std::default_random_engine gen;

	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	// Calculate new x/y for each particle
	for (int i = 0; i < num_particles; i++) {
		Particle p = particles[i];
		// If yaw rate = 0, no "turning"
		if (yaw_rate == 0) {
			p.x = p.x + cos(p.theta) * velocity * delta_t;
			p.y = p.y + sin(p.theta) * velocity * delta_t;
		} else {
			p.x = p.x + velocity / yaw_rate * (-sin(p.theta) + sin(p.theta + yaw_rate * delta_t));
			p.y = p.y + velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
		}
		// Theta will always change based on yaw_rate
		p.theta = p.theta + yaw_rate*delta_t;

		// Add noise to our results
		std::normal_distribution<double> dist_x(p.x, std_x);
		std::normal_distribution<double> dist_y(p.y, std_y);
		std::normal_distribution<double> dist_theta(p.theta, std_theta);

		// store results
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}

}


// TODO: How does this work?
// But really now?
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


	// Find out which observation is closest to the predicted measurement


	// Assign measurement to that landmark
	for (int i  = 0; i < num_particles; i++){
		// particles[i].associations;
	}
}

// Update weights based on error between measured landmarks, and theoretical landmarks.
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// for each particle
	for (int i = 0; i < num_particles; i++){
		Particle p = particles[i];
		double w = 0;
		if (observations.size() > 0){
			for (int o = 0; o < observations.size(); o++){

				LandmarkObs obs = observations[o];
				double obs_x = p.x + cos(p.theta)*obs.x - sin(p.theta)*obs.y;
				double obs_y = p.y + sin(p.theta)*obs.x + cos(p.theta)*obs.y;

				double min_distance = 1000;
				int land_id = -1;
				double x_min = 100000;
				double y_min = 100000;
				for (int m = 0; m < map_landmarks.landmark_list.size(); m++ ){
					double distance_x = map_landmarks.landmark_list[m].x_f - obs_x;
					double distance_y = map_landmarks.landmark_list[m].y_f - obs_y;
					double distance_h = sqrt(distance_x*distance_x + distance_y*distance_y);
					if (distance_h < min_distance){
						min_distance = distance_h;
						land_id = m + 1;
						x_min = obs_x;
						y_min = obs_y;
					}
				}

				// MGVPDF
				if( min_distance <= 100) {
					if (w == 0){ w = 1;}

					long double denom = 1.0 / (sqrt(abs(2.0 * M_PI * std_landmark[0] * std_landmark[1])));
					long double top = -(
									pow((x_min - map_landmarks.landmark_list[land_id - 1].x_f), 2.0) / (2.0 * pow(std_landmark[0], 2)) +
									pow((y_min - map_landmarks.landmark_list[land_id - 1].y_f), 2.0) / (2.0 * pow(std_landmark[1], 2))
					);
					//std::cout << top << " Top " << std::endl;
					w *= denom * exp(top);
				}
			}
			particles[i].weight = w;
			weights[i] = w;
			//std::cout << particles[i].weight << "Current PArticle Weight" << std::endl;
		}
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


	std::default_random_engine gen;
	std::discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;

	for (int i = 0; i < num_particles; i++){
		resample_particles.push_back((particles[distribution(gen)]));
	}

	particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
