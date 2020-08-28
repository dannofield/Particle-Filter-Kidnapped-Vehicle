# Particle-Filter-Kidnapped-Vehicle
[//]: # (Image References)

[image1]: ./images/normaldistribution.jpg "Normal Distribution"
[image2]: ./images/prediction-equations.png "Prediction Equations"
[image3]: ./images/coord_translation.png "coord translation"
[image4]: ./images/transformation_matrix.png "transformation matrix"
[image5]: ./images/homogeneous_matrix.png "homogeneous matrix"
[image6]: ./images/sensor_range.png "sensor range"
[image7]: ./images/nearestNeighbor.png "Nearest Neighbor"
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



