/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using namespace std;
/**
* init Initializes particle filter by initializing particles to Gaussian
*   distribution around first position and all the weights to 1.
* @param x Initial x position [m] (simulated estimate from GPS)
* @param y Initial y position [m]
* @param theta Initial orientation [rad]
* @param std[] Array of dimension 3 [standard deviation of x [m], 
*   standard deviation of y [m], standard deviation of yaw [rad]]
*/
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * 
   */
	num_particles = 20;// TODO: Set the number of particles to draw
	default_random_engine random;	

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	
	for (int particleIndex = 0; particleIndex < num_particles; particleIndex++)
    {
	  Particle new_particle;
	  new_particle.id = particleIndex;
	  new_particle.x = dist_x(random);
	  new_particle.y = dist_y(random);
	  new_particle.theta = dist_theta(random);
      //all weights to 1
	  new_particle.weight = 1.0;
      
      // Set of current particles
	  particles.push_back(new_particle);
      // Vector of weights of all particles
	  weights.push_back(new_particle.weight);
	}
	is_initialized = true;
}

/**
* Predicts the state for the next time step using the process model.
* @param delta_t Time between time step t and t+1 in measurements [s]
* @param std_pos[] Array of dimension 3 [standard deviation of x [m], 
*   standard deviation of y [m], standard deviation of yaw [rad]]
* @param velocity Velocity of car from t to t+1 [m/s]
* @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
	default_random_engine random;
	int i;
	for (i = 0; i < num_particles; i++) {
		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta; 

		double predicted_Xfinal;
		double predicted_Yfinal;
		double predicted_Thetafinal;	  
		//Prediction Equations
		if (fabs(yaw_rate) < 0.0001) 
		{
          	//When Yaw rate ~ cero
			predicted_Xfinal = particle_x + velocity * delta_t * cos(particle_theta);
			predicted_Yfinal = particle_y + velocity * delta_t * sin(particle_theta);
			predicted_Thetafinal = particle_theta;

		} else 
		{
			predicted_Xfinal = particle_x + (velocity/yaw_rate) * (sin(particle_theta + (yaw_rate * delta_t)) - sin(particle_theta));
			predicted_Yfinal = particle_y + (velocity/yaw_rate) * (cos(particle_theta) - cos(particle_theta + (yaw_rate * delta_t)));
			predicted_Thetafinal = particle_theta + (yaw_rate * delta_t);
		}	  
		//add random Gaussian noise
		normal_distribution<double> dist_x(predicted_Xfinal, std_pos[0]);
		normal_distribution<double> dist_y(predicted_Yfinal, std_pos[1]);
		normal_distribution<double> dist_theta(predicted_Thetafinal, std_pos[2]);	  

		particles[i].x = dist_x(random);
		particles[i].y = dist_y(random);
		particles[i].theta = dist_theta(random);
	}
}
/**
* Finds which observations correspond to which landmarks 
*   (likely by using a nearest-neighbors data association).
* @param predicted Vector of predicted landmark observations
* @param observations Vector of landmark observations

* Associate observations in map co-ordinates to predicted landmarks using nearest neighbor algorithm. 
* NOTE: the number of observations may be less than the total number of landmarks as some 
* of the landmarks may be outside the range of vehicle's sensor.
*/
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations, double sensor_range) {
  /**
   * Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
	unsigned int i, j;
	for (i = 0; i < observations.size(); i++)
	{
		//Maximum distance can be square root of 2 times the range of sensor.
		double lowest_dist = sensor_range * sqrt(2);//start with a max distance
		int closest_landmark_id = -1;
		double observation_x = observations[i].x;
		double observation_y = observations[i].y;

		for (j = 0; j < predicted.size(); j++)
		{
			double predicted_x = predicted[j].x;
			double predicted_y = predicted[j].y;
			int predicted_id = predicted[j].id;
			double current_dist = dist(observation_x, observation_y, predicted_x, predicted_y);
			//take the closest measurement as the right measurement 
			if (current_dist < lowest_dist)
			{
				lowest_dist = current_dist;
				closest_landmark_id = predicted_id;
			}
		}
		observations[i].id = closest_landmark_id;
	}
}

/**
* Updates the weights for each particle based on the likelihood
*   of the observed measurements. 
* @param sensor_range Range [m] of sensor
* @param std_landmark[] Array of dimension 2
*   [Landmark measurement uncertainty [x [m], y [m]]]
* @param observations Vector of landmark observations
* @param map Map class containing map landmarks
*/
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
	int i;
    unsigned int j;
	/*This variable is used for normalizing weights of all particles and bring them in the range
	of [0, 1]*/

	double weight_normalizer = 0.0;

	for (i = 0; i < num_particles; i++) 
	{
		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta;

		/*Transform observations from vehicle co-ordinates to map co-ordinates.*/
		//Vector containing observations transformed to map co-ordinates w.r.t. current particle.
		vector<LandmarkObs> transformed_observations;

		//Transform observations from vehicle's co-ordinates to map co-ordinates.
		for (j = 0; j < observations.size(); j++)
		{
			LandmarkObs transformed_obs;
			transformed_obs.id = j;
			transformed_obs.x = particle_x + (cos(particle_theta) * observations[j].x) - (sin(particle_theta) * observations[j].y);
			transformed_obs.y = particle_y + (sin(particle_theta) * observations[j].x) + (cos(particle_theta) * observations[j].y);
			transformed_observations.push_back(transformed_obs);
		}

		/*Filter map landmarks to keep only those which are in the sensor_range of current 
		particle. Push them to predictions vector.*/
		vector<LandmarkObs> predicted_landmarks;

		for (j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			Map::single_landmark_s current_landmark = map_landmarks.landmark_list[j];
			if ((fabs((particle_x - current_landmark.x_f)) <= sensor_range) && (fabs((particle_y - current_landmark.y_f)) <= sensor_range)) {
				predicted_landmarks.push_back(
                  LandmarkObs{
                    current_landmark.id_i,
                    current_landmark.x_f,
                    current_landmark.y_f
                    }
                );
			}
		}

		/*Associate observations with predicted landmarks using nearest neighbor algorithm.*/		
		dataAssociation(predicted_landmarks, transformed_observations, sensor_range);

		/*Calculate the weight of each particle using Multivariate Gaussian distribution.*/
		//Reset the weight of particle to 1.0
		particles[i].weight = 1.0;

		double sigma_x = std_landmark[0];
		double sigma_y = std_landmark[1];
      
		double sigma_x_2 = pow(sigma_x, 2);
		double sigma_y_2 = pow(sigma_y, 2);
		double normalizer = (1.0/(2.0 * M_PI * sigma_x * sigma_y));

		unsigned int k, l;

		/*Calculate the weight of particle based on the multivariate Gaussian probability function*/
		for (k = 0; k < transformed_observations.size(); k++)
        {
			double transformed_observation_x = transformed_observations[k].x;
			double transformed_observation_y = transformed_observations[k].y;
			double transformed_observation_id = transformed_observations[k].id;
			double multivariateGaussianProbability = 1.0;

			for (l = 0; l < predicted_landmarks.size(); l++)
            {
				double predicted_landmark_x = predicted_landmarks[l].x;
				double predicted_landmark_y = predicted_landmarks[l].y;
				double predicted_landmark_id = predicted_landmarks[l].id;
				if (transformed_observation_id == predicted_landmark_id)
                {
                  	//Weight for this observation with multivariate 
					multivariateGaussianProbability = normalizer * exp(-1.0 * ((pow((transformed_observation_x - predicted_landmark_x), 2)/(2.0 * sigma_x_2)) + (pow((transformed_observation_y - predicted_landmark_y), 2)/(2.0 * sigma_y_2))));
                  
                  	//Product of this obersvation weight with total
					particles[i].weight *= multivariateGaussianProbability;
				}
			}
		}
		weight_normalizer += particles[i].weight;
	}
	/*Normalize the weights of all particles  using probabilistic approach.*/
	for (unsigned int i = 0; i < particles.size(); i++)
	{
		particles[i].weight /= weight_normalizer;
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

	vector<Particle> resampled_particles;
	// Create a generator to be used for generating random particle index and beta value
	default_random_engine gen;

	//Generate random particle index
	//uniform_int_distribution<int> particle_index(0, num_particles - 1);
	discrete_distribution<int> particle_index(0, num_particles - 1);
  
	int current_index = particle_index(gen);
	double beta = 0.0;
	double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());
	for (unsigned int i = 0; i < particles.size(); i++)
    {
		uniform_real_distribution<double> random_weight(0.0, max_weight_2);
		beta += random_weight(gen);
		while (beta > weights[current_index])
        {
			beta -= weights[current_index];
			current_index = (current_index + 1) % num_particles;
		}
		resampled_particles.push_back(particles[current_index]);
	}
	particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
//https://github.com/udacity/CarND-Kidnapped-Vehicle-Project
//https://github.com/sohonisaurabh/CarND-Kidnapped-Vehicle-Project
//https://medium.com/intro-to-artificial-intelligence/kidnapped-vehicle-project-using-particle-filters-udacitys-self-driving-car-nanodegree-aa1d37c40d49
/*
cd CarND-Kidnapped-Vehicle-Project
./clean.sh
./build.sh
./run.sh
*/