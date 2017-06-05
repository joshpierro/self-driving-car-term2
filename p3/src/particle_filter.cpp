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
#include <random>

#include "particle_filter.h"
#include "map.h"

using namespace std;

//random number engine class that generates pseudo-random numbers
default_random_engine generator;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	//Set the number of particles
	num_particles = 151;
	cout << "Initializing " << num_particles << " Particles!" << endl;
	cout << "With Starting position of  " << x << "," << y << endl;
	cout << "and theta of  " << theta << endl;

	// set normal distributions using GPS measurement uncertainty
	normal_distribution<double> normal_distribution_x(0, std[0]);
	normal_distribution<double> normal_distribution_y(0, std[1]);
	normal_distribution<double> normal_distribution_theta(0, std[2]);

	//Create and initialize all particles to first position,
	//add random Gaussian noise to each particle set all weights to 1
	for (int i = 0; i <= num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.x = x + normal_distribution_x(generator);
		particle.y = y + normal_distribution_y(generator);
		particle.theta = theta + normal_distribution_theta(generator);
		particle.weight = 1;
		particles.push_back(particle);
	}

	//todo assert values fall within the standard deviation

	is_initialized = true;
	cout << "Initialized!" << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
		double velocity, double yaw_rate) {

	//normal distributions for noise
	normal_distribution<double> normal_distribution_x(0, std_pos[0]);
	normal_distribution<double> normal_distribution_y(0, std_pos[1]);
	normal_distribution<double> normal_distribution_theta(0, std_pos[2]);

	for (Particle& particle : particles) {
		//add measurments
		//update x, y and the yaw angle when the yaw rate is not equal to zero:
		if (fabs(yaw_rate) != 0) {
			particle.x = particle.x + (velocity / yaw_rate)	* (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
			particle.y = particle.y + (velocity / yaw_rate) * (cos(particle.theta) - cos(particle.theta + (yaw_rate * delta_t)));
			particle.theta += yaw_rate * delta_t;
		}

		//add random Gaussian noise
		//update particles with noise
		particle.x += normal_distribution_x(generator);
		particle.y += normal_distribution_y(generator);
		particle.theta += normal_distribution_theta(generator);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	//loop through the observations
	for (LandmarkObs& observation : observations) {

		//initialize distance to max double
		double distance = std::numeric_limits<double>::max();

		//variable to hold best  prediction
		LandmarkObs bestPrediction;

		//loop through predictions
		for (LandmarkObs prediction : predicted) {
			//if the distance betwix the observation and the prediction is less than the distance variable
			//set the observation id to the prediction id
			if (dist(observation.x, observation.y, prediction.x,prediction.y) < distance) {
				distance = dist(observation.x, observation.y,prediction.x, prediction.y);
				bestPrediction = prediction;
			}
			observation.id = bestPrediction.id;
		}
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {

	//loop through particles
	for (Particle& particle : particles) {

		// create a collection to hold landmarks within sensor range
		vector<LandmarkObs> landmarks_within_range;

		//loop through map landmarks
		for (unsigned int i = 0; i < map_landmarks.landmark_list.size(); i++) {
			//if the difference between the landmark x and y coords is <= sensor range,
			//add it to the landmarks within range collection
			if (dist(map_landmarks.landmark_list[i].x_f,map_landmarks.landmark_list[i].y_f, particle.x, particle.y) <= sensor_range) {
				//hungarian notation WTF?!!
				landmarks_within_range.push_back(
						LandmarkObs { map_landmarks.landmark_list[i].id_i,
								map_landmarks.landmark_list[i].x_f,
								map_landmarks.landmark_list[i].y_f });
			}
		}

		//collection to hold transformed landmarks
		vector<LandmarkObs> transformed_landmark_observations;

		//transform observation coordinates to map coordinates
		for (LandmarkObs& landmark_observation : observations) {
			double transformed_x = cos(particle.theta) * landmark_observation.x - sin(particle.theta) * landmark_observation.y + particle.x;
			double transformed_y = sin(particle.theta) * landmark_observation.x + cos(particle.theta) * landmark_observation.y + particle.y;
			LandmarkObs transformed_landmark = { landmark_observation.id,transformed_x, transformed_y };
			transformed_landmark_observations.push_back(transformed_landmark);
		}

		//associate landmarks within range predictions and transformed observations
		dataAssociation(landmarks_within_range, transformed_landmark_observations);

		//reset particle weight to 1
		particle.weight = 1;

		//loop through the transformed observations
		for (LandmarkObs& landmark_observation : transformed_landmark_observations) {

			//get the prediction associated to the (transformed) observation
			LandmarkObs associatedPrediction;
			//TODO - refactor : I feel like this would be better using the higher order function filter
			for (LandmarkObs& landmark_within_range : landmarks_within_range) {
				if (landmark_within_range.id == landmark_observation.id) {
					associatedPrediction = landmark_within_range;
				}
			}

			//calculate and set weight
			//TODO - this should be a helper method
			double obs_w = (1 / (2 * M_PI * std_landmark[0] * std_landmark[1]))* exp(
							-(pow(
									associatedPrediction.x
											- landmark_observation.x, 2)
									/ (2 * pow(std_landmark[0], 2))
									+ (pow(
											associatedPrediction.y
													- landmark_observation.y, 2)
											/ (2 * pow(std_landmark[1], 2)))));
			particle.weight *= obs_w;
		}
	}
}

void ParticleFilter::resample() {

	//create a collection to hold weights
	vector<double> weights;

	//add particle weights to collection :todo use functional programming to derive this collection
	for (Particle& particle : particles) {
		weights.push_back(particle.weight);
	}

	//create discrete distribution
	discrete_distribution<> distribution(weights.begin(), weights.end());

	//create a collection for resampled particles
	vector<Particle> resampled_particles;

	//add weighted particles to resampled particle collection
	for (int i = 0; i < particles.size(); i++) {
		resampled_particles.push_back(particles[distribution(generator)]);
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
