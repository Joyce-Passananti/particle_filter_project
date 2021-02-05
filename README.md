# particle_filter_project

## Team Member Names: Daria Shifrina, Joyce Passananti 

## Implementation Plan:

### Initializing Particle Cloud
- The initial particle cloud is equal to the first collected PoseArray. If we want to start with a particle cloud of 100 particles, we will take the length of the PoseArray, divide it by the amount of particles (100 in this case), and then pick every 100th Pose in the PoseArray, initialize it as a particle object and add it to the particle cloud array. This will create a uniform spread of particles over the array.
- To test whether it picked a uniform distribution, we will create the particle array, add it to the map, and then use self.get_map() to see whether they are uniformly spread out over the map. 

### Updating Particle Positions
- We will calculate the distance traveled (in the forward direction) by storing the robot scan sensor data before and after it moves, and calculating the difference between the two values on the axis moved along. We will then iterate through our particle cloud and modify each position by the calculated value in the determined direction.
- We will test out using a small sample of particles to begin with, and visually reference the map to check the particles seem to be moving in the correct direction.

### Updating Particle Weights
- Using the formula introduced in the class exercise:  
  ![equation](/equation.png)  
  We will calculate the left, middle, and right “sensor data” given each particle’s new position, and compare zt[m][i] where i[3]for each particle m to the measured sensor data from the robot scan zt[i]. 
- We will test out using a small sample of particles to begin with, and visually reference the map to check the weights calculated are representative of what we see, and show a pattern converging. 

### Normalizing Particles
- To reweight particles, we will iterate through the particle cloud and for each particle p that has weight w_p, we will compute the normalized weight by dividing individual w_p by the sum of weights from all particles.
- To resample particles, we will call draw_random_sample(choices, probabilities, n) and pass in the following:
    - Choices: our current array self.particle_cloud
    - Probabilities: the normalized_weights
    - N: Length of self.particle_cloud 
- We will test using print functions and compare with our own calculations/observations.To facilitate this manual testing we will start out with a small particle cloud.

### Updating Robot Pose
- The estimated pose of the robot will mirror the position of the particle with the highest weight in the particle_cloud array. The particle with the highest weight is the one that matches the robot’s current LaserScan data the best, therefore it is the best estimate of its actual current location. To implement it, we will sort through the current particle cloud for the highest weighted particles. 
- To test this, we will simply check whether the robot position matches the position of the particle with the highest weight.

### Incorporating Noise
- To minimize noise for the robot movement, we will use the robot’s LaserScan data to approximate how much it actually moves forward before updating the particle positions. For instance, if we aim to move the robot forward one step, we will store its LaserScan data for the nearest object directly in front of it. Then, after it moves forward that one step, we will get the difference between its current LaserScan data directly in front with the previous. Therefore, if it moves a little over a step, then we will move all the particles forward by that amount.
- Additionally, in the final step where we predict where our robot is, we will not look for a specific matching particle but a particle whose hypothetical LaserScan data is within 2-3 degrees of the robot’s actual LaserScan data.


## Timeline:
Sunday 1/31: finish initialize_particle_cloud() and update_particles_with_motion_model()  
Wednesday ⅔ : finish update_particle_weights_with_measurement_model(), normalize_particles()  
Friday ⅖:  resample_particles() and update_estimated_robot_pose().  
Sunday 2/7: Figure out way out of room. Finalize code. Finalize gif.   
Tuesday 2/9: Finalize writeup. Submit it.  
