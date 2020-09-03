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

### Normal Distribution (or Gaussian or Gauss or Laplace–Gauss) distribution
![alt text][image1]

# Prediction Equations
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

Matrix multiplication results in:

![alt text][image5]

### Filter map landmarks to keep only those which are in the sensor_range
![alt text][image6]

### Associate observations to predicted landmarks using nearest neighbor algorithm
![alt text][image7]

# Update Weights
### Multivariate-Gaussian probability density
![alt text][image8]

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
