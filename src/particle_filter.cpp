/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */
#define _USE_MATH_DEFINES

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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set number of particles
	num_particles = 750;

	// Set Initial Particle Positions from GPS cords
	default_random_engine gen;
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// Set Global (Map-wise) position
	for (int i = 0; i < num_particles; i++){
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);

		// Add to Particles Vector
		particles.push_back(p);
		weights.push_back(0);
	}

	is_initialized = true;

}



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	std::default_random_engine gen;
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	double std_v = sqrt(std_x*std_x + std_y*std_y);

	// Add Noise
	std::normal_distribution<double> dist_v(velocity, std_v);
	std::normal_distribution<double> dist_theta(yaw_rate, std_theta);

	// for every particle
	for (int i = 0; i < num_particles; i++){
		// take current position
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		//double samp_vel = dist_v(gen);
		//double samp_theta = dist_theta(gen);
		double samp_vel = velocity;
		double samp_theta = yaw_rate;

		// apply velocity and yaw rate to it
		double x_change;
		double y_change;
		if (fabs(samp_theta) > 0.0001){
			double scaler = samp_vel/samp_theta;
			x_change = x + scaler*(-sin(theta) + sin(theta + samp_theta*delta_t));
			y_change = y + scaler*(cos(theta) - cos(theta + samp_theta*delta_t));
		}
		else{
			x_change = x + cos(theta)*samp_vel*delta_t;
			y_change = y + sin(theta)*samp_vel*delta_t;
		}
		double theta_change = theta + samp_theta*delta_t;


		std::normal_distribution<double> dist_x(x_change, std_x);
		std::normal_distribution<double> dist_y(y_change, std_y);
		//std::normal_distribution<double> dist_theta(theta_change, std_theta);

		// return new predicted location/yaw

		//particles[i].x = dist_x(gen);
		//particles[i].y = dist_y(gen);
		//particles[i].theta = dist_theta(gen);

		//std::cout << theta_change << endl;

		particles[i].x = x_change;
		particles[i].y = y_change;
		particles[i].theta = theta_change;

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

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


	for (int i = 0; i < num_particles; i++) {
		double par_x = particles[i].x;
		double par_y = particles[i].y;
		double par_theta = particles[i].theta;
		long double weight = 0;
		double std_land_x = std_landmark[0];
		double std_land_y = std_landmark[1];

		// If there are Observations
		if ((observations).size() > 0) {

			// Create a Best Landmark Matching List
			vector<double> best_land_list(2*map_landmarks.landmark_list.size());
			for (int s = 0; s < best_land_list.size(); s++){
				best_land_list[s] = 100;
			}

			// For each observation
			for (int j = 0; j < observations.size(); j++) {
				// grab observation x/y
				LandmarkObs obs = observations[j];
				double x = obs.x;
				double y = obs.y;

				// Check if Observation is within sensor range
				if (sqrt(pow(x, 2) + pow(y, 2)) < sensor_range) {
					// Convert Landmark pos from Observation to Particle pos
					double x_est = par_x + cos(par_theta) * x - sin(par_theta) * y;
					double y_est = par_y + sin(par_theta) * x + cos(par_theta) * y;

					// Associate with nearest landmark
					int id = -1;
					double min = 100;
					// Check each landmark for distance between landmark and observation
					for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
						Map::single_landmark_s landmark = map_landmarks.landmark_list[k];
						double land_x = landmark.x_f;
						double land_y = landmark.y_f;
						int land_id = landmark.id_i;
						double x_dist = land_x - x_est;
						double y_dist = land_y - y_est;
						double land_distance = sqrt(pow(y_dist, 2) + pow(x_dist, 2));
						if (land_distance < min) {
							min = land_distance;
							id = land_id;
						}
					}
					// Store most accurate landmark measurements in best list
					if (id > -1) {
						Map::single_landmark_s landmark = map_landmarks.landmark_list[id - 1];
						double land_x = landmark.x_f;
						double land_y = landmark.y_f;
						double inc_x = best_land_list[2*id - 2];
						double inc_y = best_land_list[2*id - 1];
						double inc_distance = sqrt(pow(land_x - inc_x, 2) - pow(land_y - inc_y, 2));
						double chal_distance = sqrt(pow(land_x - x, 2) - pow(land_y - y, 2));
						if (chal_distance < inc_distance) {
							best_land_list[2*id - 2] = x;
							best_land_list[2*id - 1] = y;
						}
					}
				}
			}

			// calculate weights for all best landmark estimates
			for ( int t = 2; t < best_land_list.size(); t +=2) {
				// if landmark estimate exists
				if (best_land_list[t] < 100){
					int id = t/2;
					Map::single_landmark_s landmark = map_landmarks.landmark_list[id - 1];
					double land_x = landmark.x_f;				// our landmark coord
					double land_y = landmark.y_f;				// our landmark coord
					double x_est = best_land_list[2*id - 2];	// our observation coord 	// could have used t - 2
					double y_est = best_land_list[2*id - 1];	// our observation coord 	// could have used t - 1

					//MVGPDF
					double denom = 1.0/(sqrt(abs(2.0*M_PI*std_land_x*std_land_y)));
					double top = -(pow((x_est - land_x),2.0)/(2.0*std_land_x*std_land_x) + pow((y_est - land_y),2.0)/(2.0*std_land_y*std_land_y));
					long double w = denom*exp(top);
					weight *= w;
				}
			}
		}
			// set particle weight
			particles[i].weight = weight;
			weights[i] = weight;
	}

}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//double highest_weight = 0.0;
	long double highest_weight = *max_element(weights.begin(), weights.end());

	cout << "Highest Weight = " << highest_weight << endl;
	std::vector<Particle> resampled_particles;
	int index = 0 + (rand() % static_cast<int>(num_particles - 0 + 1));
	double beta = 0.0;

	for (int i = 0; i < num_particles; i++){
		beta += (((double)rand() / (RAND_MAX)) *2.0 *highest_weight);
		while (beta > weights[index]){
			beta -= weights[index];
			index = (index +1 )% num_particles;
		}
		resampled_particles.push_back(particles[i]);
	}
	particles = resampled_particles;


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
