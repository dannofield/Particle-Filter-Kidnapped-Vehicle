# Particle-Filter-Kidnapped-Vehicle
[//]: # (Image References)

[image1]: ./images/normaldistribution.jpg "Normal Distribution"
[image2]: ./images/prediction-equations.png "Prediction Equations"
### Normal Distribution (or Gaussian or Gauss or Laplace–Gauss) distribution
![alt text][image1]

### Prediction Equations
![alt text][image2]

```
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
