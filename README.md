# Particle-Filter-Kidnapped-Vehicle
[//]: # (Image References)

[image1]: ./images/normaldistribution.jpg "Normal Distribution"
[image2]: ./images/prediction-equations.png "Prediction Equations"
[image3]: ./images/coord_translation.png "coord translation"
[image4]: ./images/transformation_matrix.png "transformation matrix"
[image5]: ./images/homogeneous_matrix.png "homogeneous matrix"
[image6]: ./images/sensor_range.png "sensor range"
[image7]: ./images/nearestNeighbor.png "Nearest Neighbor"
[image8]: ./images/Multivariate-Gaussian.png "Multivariate Gaussian"
[image9]: ./images/ParticleFilter.png "Particle Filter"

# Project Introduction
A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project we will implement a 2 dimensional particle filter in C++. Our particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

# Step 1: Normal Distribution (or Gaussian or Gauss or Laplace–Gauss) distribution

We initialize a particle filter by initializing particles to Gaussian distribution around first position and all the weights to 1.

Cpp has the tools to apply a Normal/Gaussian distribution [normal_distribution](http://en.cppreference.com/w/cpp/numeric/random/normal_distribution) using a [default_random_engine](http://www.cplusplus.com/reference/random/default_random_engine/)

Also, for each position received, we will simulate noisy measurement from GPS sensor. This measurement includes the x coordinate, y coordinate (both in m) and the theta (orientation) of vehicle in radian. 

This noise is modelled by Gaussian distribution with standard deviation in x, y and theta provided as a part of GPS uncertainty specification.

![alt text][image1]

```Cpp
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// :
	default_random_engine random;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	// :	
	for (int particleIndex = 0; particleIndex < num_particles; particleIndex++)
    	{
	  // :
	  particle.x = dist_x(random);
	  particle.y = dist_y(random);
	  article.theta = dist_theta(random);
```

# Step 2: Prediction Equations

After initialization, the particle filter gets updated with information of control inputs like magnitude of velocity (v) and yaw rate (θ), and time elapsed between steps. 

With this information, the location of each particle at next time step is predicted. 

Location update is done with the help of the formulas:

![alt text][image2]

```Cpp
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
```

# Transformation
![alt text][image3]

### Homogenous Transformation Matrix
![alt text][image4]

Observations in the car coordinate system can be transformed into map coordinates (xm and ym) by passing car observation coordinates (xc and yc), map particle coordinates (xp and yp), and our rotation angle (-90 degrees) through a homogenous transformation matrix. This homogenous transformation matrix, performs rotation and translation.

Matrix multiplication results in:

![alt text][image5]

```Cpp
transformed_obs.x = particle_x + (cos(particle_theta) * observations[j].x) - (sin(particle_theta) * observations[j].y);
transformed_obs.y = particle_y + (sin(particle_theta) * observations[j].x) + (cos(particle_theta) * observations[j].y);
```

### Filter map landmarks to keep only those which are in the sensor_range
![alt text][image6]

### Associate observations to predicted landmarks using nearest neighbor algorithm

We have to match landmark measurements to objects in the real world provided as map landmarks

![alt text][image7]

```Cpp
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
```

# Update Weights
### Multivariate-Gaussian probability density

After we have done the measurement transformations and associations, we need to calculate the final weight of the particle.
The Multivariate-Gaussian standard deviation is described by our initial uncertainty in the x and y ranges and its association with the landmark's position

![alt text][image8]

```Cpp
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
			multivariateGaussianProbability = normalizer * 
			exp(-1.0 * 
				(
					(pow((transformed_observation_x - predicted_landmark_x), 2)/(2.0 * sigma_x_2)) + 
					(pow((transformed_observation_y - predicted_landmark_y), 2)/
						(2.0 * sigma_y_2))
				)
			);

			//Product of this obersvation weight with total
			particles[i].weight *= multivariateGaussianProbability;
		}
	}
}
```


# Project Results
![alt text][image9]

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Our job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

### Documentation

Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age [https://arxiv.org/abs/1606.05830](https://arxiv.org/abs/1606.05830)

Navigating the Landscape for Real-time Localisation and Mapping for Robotics and Virtual and Augmented Reality [https://arxiv.org/abs/1808.06352](https://arxiv.org/abs/1808.06352)

Robotic Mapping: A Survey [http://robots.stanford.edu/papers/thrun.mapping-tr.pdf](http://robots.stanford.edu/papers/thrun.mapping-tr.pdf)

Udacity / CarND-Kidnapped-Vehicle-Project [https://github.com/udacity/CarND-Kidnapped-Vehicle-Project](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)

Kidnapped vehicle project using Particle Filters-Udacity’s Self-driving Car Nanodegree

https://github.com/sohonisaurabh/CarND-Kidnapped-Vehicle-Project

https://medium.com/intro-to-artificial-intelligence/kidnapped-vehicle-project-using-particle-filters-udacitys-self-driving-car-nanodegree-aa1d37c40d49

See here http://users.isy.liu.se/rt/fredrik/reports/09TAESpftutorial.pdf where theory meets practice in particle filters
http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf

Applying particle filters in robots http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf

A gentle introduction to particle filtering http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf

Particle filters and its applications http://ocw.alfaisal.edu/NR/rdonlyres/Aeronautics-and-Astronautics/16-412JSpring-2005/F9652688-E118-442E-98CE-3013CBEB8F11/0/a5_hso_plnvl_mlr.pdf

Get to understand the different particle filter has with other filters here http://www.dsi.unifi.it/users/chisci/idfric/Nonlinear_filtering_Chen.pdf

Robot mapping with particle filters http://duch.mimuw.edu.pl/~kowaluk/GOBR/slam11-particle-filter.pdf
https://en.wikipedia.org/wiki/Particle_filter

[Kalman and Particle filters](https://docs.google.com/viewer?url=http%3A%2F%2Fwww.sft.asso.fr%2FLocal%2Fsft%2Fdir%2Fuser-3775%2Fdocuments%2Factes%2FMetti5_School%2FLectures%26Tutorials-Texts%2FText-T10-Orlande.pdf)

[Particle/Kalman Filter for Efficient Robot Localization](https://docs.google.com/viewer?url=http%3A%2F%2Fresearch.ijcaonline.org%2Fvolume106%2Fnumber2%2Fpxc3899554.pdf)
