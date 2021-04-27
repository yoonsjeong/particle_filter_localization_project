# particle_filter_localization_project

## Writeup

### Objectives Description
The goal of this project is to implement a working implementation of Monte Carlo Localization, an algorithm that determines the location of a robot given a scanner and a map of the region the robot will be traversing. MCL makes use of a "particle cloud" that is adjusted probabilistically which allows the robot to adjust for sensor noise and unpredictable events such as slippage. This algorithm is useful as once the robot discovers its location, it becomes far easier to navigate around its surroundings.

### High-Level Description
At a high level, we solved robot localization using five main steps. First, we initialized the particle cloud — a random scattering of points all around the entire map. Next, once the robot began to move and generated odometry data, we updated all the random particles positions appropriately. Then, we ran the likelihood field range finder algorithm to calculate a weight for each particle. We normalized those weights, and then resampled, choosing the most probable particles and creating a new particle cloud. This cycle repeats until we are left with a localized distribution of particles surrounding the actual robot as it moves. 

### Main Steps
The following are the 7 main steps for Monte Carlo Localization:
#### 1. Initialization of Particle Cloud
1. We implemented the initialization of the particle cloud in the functions `get_particle` and `initialize_particle_cloud`
2. The get_particle function randomly selects grid coordinates in the map data structure, and checks if they are valid or not. If they are, we use a formula to convert the grid points to valid map points. Then, in the intialize_particle_cloud function, we use these valid coordinates and create particles using the map coordinates, randomized theta (orientation values), and weights of 1.
#### 2. Movement Model
1. We implemented the movement model in the function `update_particles_with_motion_model` 
2. We define a for loop that iterates through each particle in the particle cloud. For each particle, we calculate the new_yaw, a value that is the sum of the particles orientation and the change in the yaw from the odometry. We use this new_yaw to project the changes in the x and y odometry onto the new particle. For the x update, we add to the old x position the change in x from the odometry times the cos of the new_yaw. We do the same for y but with sin. 
#### 3. Measurement model
1. We implemented the measurement model in the function `update_particle_weights_with_measurement_model` 
2. We initiate a for loop that iterates through each particle in the cloud. For each particle, we define another for loop that iterates through 8 angles collected by the LIDAR. For each angle, we calculate a q value for the particle based on the likelihood finder algorithm, and multiply the q values together to get the particle weight update. 
#### 4. Resampling
1. We implemented resampling in the functions `normalize particles`, `resample_particles`, and `draw_random_samples`
2. After normalizing the particle by summing the weights and dividing each weight by the sum, we plug it into the function `draw_random_samples`. This function uses a handy statistics method -- first, it adds up all the probabilities and forms an accumulation list, or a list of bins. After it does this, it samples a uniform distribution from 0 to 1 and picks the first bin that is greater than the sample taken. We do this for the number of particles there are total to completely resample. 
#### 5. Incorporation of noise
1. We incorporated noise in the `update_particles_with_motion_model` function, and by playing with the standard deviation, also in the `update_particle_weights_with_measurement_model` function
2. In the motion model function, we added noise to every position and orientation update. In the measurement model function, by increasing the standard deviation on the function call `compute_prob_zero_centered_gaussian`, we increase the acceptable range of values which can be considered accurate, thereby increasing the noise for the model. 

#### 6. Updating estimated robot pose 
1. We implement updating the estimated robot pose in the function update_estimated_robot_pose
2. In this function, we sum up the x,y, and yaw values from all the particles. Then, we calculate the average x,y, and yaw and assign that to the robot_estimate variable as a new pose. As the particle cloud contains more and more accurate particles, this estimate becomes closer and closer to the true position and orientation of the robot. 
#### 7. Optimization of parameters
1. We had to optimize parameters in get_particles, and spent time experimenting with the total number of particles assigned to the variable num_particles. 
2. In `get_particles`, we experimented with several different formulas for computing the transformation between grid coordinates and map coordinates. Our initial formula involved subtracting the origin of the grid map from the x,y coordinates of the grid, and then multiplying by the resolution. This resulted in a few stray particles outside the bounds of the box, and so we ended up implementing a different conversion which had no stray coordinates. We also experimented with different numbers of particles, and found that using 5000 particles was easier to track and quicker than using 10000. Finally, we also experimented with the different noise values listed above. 


### Challenges
We had a number of issues with the NoMachine failing to work due to issues with Jonah's unstable internet connection. Luckily, Jonah communicated his problems with the TAs as soon as this happened and we were able to get a one day extension. We also had a major challenge with a bug that none of us could track down regardless of how many print statements we put in. The bug was that the weights were being altered even though we did not adjust them. After consulting with Yves we suspected the bug may have been caused by a pass-by-reference error, in which "pointers" to particles were being copied, meaning when we altered one particle it would alter numerous particles. To fix this bug, we made sure to construct a new Particle class in any instance where we were altering the particle cloud. We also had a big error from relying on a numpy function to do our random sampling for us - the numpy function was returning particles that had zero weights, and that tripped us up for a while as well. 

### Future Work
If we had more time, we might have implemented the full version of the likelihood algorithm from class, including the Z hit, Random, and Z max values, and then compared the performance to our current algorithm. We might have also then attempted to implement an alternative method for updating the particle weights, like the beam range finder method. Generating performance statistics (speed, number of particles necessary for efficiency, etc) to compare these different methods would be useful too. 


### Takeaways
* Don't assume that the measurement model handles everything! We thought initially that the measurement model would end up zeroing out the weights of particles that had inaccurate positions and orientations, and so it wasn't so important to get the projections right for the particle updates in the motion models. This wasn't true! Even when resampling worked properly, it was important to really think through how to accurately project the odometry changes onto our particle positions. 
* Don't assume Python handles everything. The biggest takeaway for me was to be knowledgeable about how Python functions handle objects within lists. Some care must be taken to avoid cryptic bugs such as the pass-by-reference bug described earlier. Python is certainly easier to use than C, but pointers still exist, to some extent! 
* Don't assume Python built in functions work as you expect them to. We spent a lot of time wondering why our resampling didn't work, only to find that the python numpy function we were relying on just wasn't working. Next time, we might start out by just writing our own functions instead of relying on a function we aren't so familiar with. 



## Implementation Plan

1. The names of your team members
Jonah Kaye, Yoon Jeong

1. How you will initialize your particle cloud (initialize_particle_cloud)?
	
    **Implementation:** We will generate uniformly random points and put it into an array called `self.particle_cloud` using the Python "random" library. We will use a for loop to do this. We will use as many random points as possible without compromising on efficiency. 

    **Test:** We will do an eye check by printing the variable (using "echo") to make sure that it appears random and is within the bounds of what we have set. We can print the length of the array to ensure that it has the number of initial points we expect.
1. How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
	
    **Implementation:** We will subscribe to the odometer rostopic and add the distance travelled to the particles in the particle cloud, and will also add Gaussian noise by using the Python random library. We will use a for loop to iterate through the particle cloud.
	
    **Test:** We will instruct our robot to go a certain distance, and as above, echo our particle cloud array variable and do an eyeball check to see that the points have been updated.
1. How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?

    **Implementation:** We will subscribe to the scan rostopic and convert the ms.ranges into the z_t(m) value. Then, we will use the measurement weight equation from class, taking the sum of 1/abs(Z_t(i) - Z_t(m)) and then we will put it into an ordered weight list that is stored in the class.

    **Test:** We will take an example, having instructed our robot to go only a certain distance, and then check by hand that the robot calculations are correct. 

1. How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
	
    **Implementation:** To normalize the weights of the particle, we will sum up all the particle weights and then take the inverse, and then multiply each weight by that inverse. We will use a for loop to iterate through the particle weights. There is a python function in the "random" library called `choices` that will allow us to select weights to resample with the appropriate probabilities.
	
    **Test:** To test that normalization worked properly, we can check to see that the list sums up to 1. To see if the resampling is working properly we can see if the visualization we create is converging upon certain peaks and removing a good number of particles per time step.
1. How you will update the estimated pose of the robot (update_estimated_robot_pose)?

    **Implementation:** We will take the average of all the points that have been resampled in order to find the estimated pose.

    **Test:** To test this, we will put it into rviz and do an eyeball check to see that the pose approximates the cloud of particles.
1. How you will incorporate noise into your particle filter localization?
	
    **Implementation:** We will be incorporating noise by using the normal function available from the Python random library.
	
    **Test:** To test this, we will see if rviz produces a cloud of particles rather than slowly converging on individual points.

1. A brief timeline sketching out when you would like to have accomplished each of the components listed above.

**By this weekend:** We want to be fully finished with `initialize_particle_cloud` (Yoon) and `update_particles_with_motion_model` (Jonah). We want to have started `update_particle_weights_with_measurement_model` (Yoon) and `normalize_particles` (Jonah) and `resample_particles` (Jonah).
**By next Wednesday:** We want to be finished with writing all the coding aspects and be working on debugging.
This timeline may change as we work on it.

