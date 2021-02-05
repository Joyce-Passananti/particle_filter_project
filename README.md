
# Particle Filter Project
**Team Member Names**: Daria Shifrina, Joyce Passananti  

<p align="center">
  <img src="particlefilter.gif" alt="Particle Filter Demo"/>
</p>

## Implementation Plan

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


## Timeline

1. Sunday 1/31: finish initialize_particle_cloud() and update_particles_with_motion_model()  
2. Wednesday 2/3 : finish update_particle_weights_with_measurement_model(), normalize_particles()  
3. Friday 2/5:  resample_particles() and update_estimated_robot_pose().  
4. Sunday 2/7: Figure out way out of room. Finalize code. Finalize gif.   
5. Tuesday 2/9: Finalize writeup. Submit it.  

## Writeup

### Objective Description

The goal of this project is to implement a particle filter algorithm to solve a robot localization problem. In practical situations, when robots are exploring unfamiliar terrain, or otherwise trying to reconfigure their location awareness, a particle filter algorithm narrows down the location of the robot by making particles emulate a robot's position.

### High-level Description

The particle filter project consists of emulating made up particles that mirror the movements of the robot and then narrowing down the particles that most resemble the sensor readings of the bot. First, within the provided map, generate 19500 randomly placed particles. To make sure they are properly placed, make sure they are within the boundary constraints of the maps and do not intersect with any obstacles. When the robot moves, the particle's positions are updated. This is done by calculating the current and previous position of the robot, finding the distance moved and the point of rotation. Each particle is replaced with a new Pose object, whose position is moved and updated with the robot's orientation. Then, the weights of the particles are calculated based on how closely their simulated sensor readings reflect the actual sensor readings of the robot. Based on these weights, a new round of particles are remodeled so that the higher weighted particles are more represented. Within multile time steps, these particles narrow down on the actual location of the robot. Additionally, the estimated robot pose is constantly updated on the map as well to reflect the average x/y positions and orientation of the particles. 

### Code Breakdown

#### Movement

- Code location: (1-3 sentences): Please describe where in your code you implemented this step of the particle filter.
- Function/code description: (1-3 sentences per function / portion of code): Describe the structure of your code. For the functions you wrote, describe what each of them does and how they contribute to this step of the particle filter.

#### Computation of Importance Weights

- Code location: (1-3 sentences): Please describe where in your code you implemented this step of the particle filter.
- Function/code description: (1-3 sentences per function / portion of code): Describe the structure of your code. For the functions you wrote, describe what each of them does and how they contribute to this step of the particle filter.

#### Resampling

- Code location: (1-3 sentences): Please describe where in your code you implemented this step of the particle filter.
- Function/code description: (1-3 sentences per function / portion of code): Describe the structure of your code. For the functions you wrote, describe what each of them does and how they contribute to this step of the particle filter.

### Challenges

(1 paragraph): If you had more time, how would you improve your particle filter?

### Future Work

(1 paragraph): If you had more time, how would you improve your particle filter?

###  Takeaways

(at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.

