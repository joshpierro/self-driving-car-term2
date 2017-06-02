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

using namespace std;

//random number engine class that generates pseudo-random numbers
default_random_engine generator;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	//Set the number of particles
	num_particles = 100;
	cout << "Initializing " << num_particles << " Particles!" << endl;
	cout << "With Starting position of  " << x << "," << y << endl;
	cout << "and theta of  " << theta << endl;

	// set normal distributions using GPS measurement uncertainty
	normal_distribution<double> normal_distribution_x(x, std[0]);
	normal_distribution<double> normal_distribution_y(y, std[1]);
	normal_distribution<double> normal_distribution_theta(theta, std[2]);

	//Initialize all particles to first position, add random Gaussian noise to each particle set all weights to 1
	for (int i = 0; i <= num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.x = normal_distribution_x(generator);
		particle.y = normal_distribution_y(generator);
		particle.theta = normal_distribution_theta(generator);
		particle.weight = 1;
		particles.push_back(particle);
	}

	for (int i = 0; i <= num_particles; i++) {
	 cout << particles[i].x << endl;
	 cout << particles[i].y << endl;
	 cout << particles[i].theta << endl;
	 cout << "------------------" << endl;
	 }

	//todo assert values fall within the standard deviation

	is_initialized = true;
	cout << "Initialized!" << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	for (int i = 0; i <= num_particles; i++) {

		double  x, y, theta;

		//add measurments
		//update x, y and the yaw angle when the yaw rate is not equal to zero:
		if (yaw_rate != 0) {
			particles[i].x = particles[i].x + (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y = particles[i].y + (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate*delta_t)));
			particles[i].theta += yaw_rate * delta_t;;
		} else {
			particles[i].x = velocity * delta_t * cos(particles[i].theta);
			particles[i].y = velocity * delta_t * sin(particles[i].theta);
		}


		//add random Gaussian noise

		//normal distributions for noise
		normal_distribution<double> normal_distribution_x(0, std_pos[0]);
		normal_distribution<double> normal_distribution_y(0, std_pos[1]);
		normal_distribution<double> normal_distribution_theta(0, std_pos[2]);

		//update particles
		particles[i].x += normal_distribution_x(generator);
		particles[i].y += normal_distribution_y(generator);
		particles[i].theta += normal_distribution_theta(generator);

	}

	/*for (int i = 0; i <= num_particles; i++) {
	 cout << particles[i].x << endl;
	 cout << particles[i].y << endl;
	 cout << particles[i].theta << endl;
	 cout << "------------------" << endl;
	 }*/


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
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
