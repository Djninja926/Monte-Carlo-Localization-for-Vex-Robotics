# Monte Carlo Localization for Vex Robotics

#### Credit to $${\color{purple} Maxx \space Wilson | GHOST}$$, $${\color{red} Benjamin \space | 687D}$$, and [Autonomous Mobile Robotics Laboratory](https://amrl.cs.utexas.edu/interactive-particle-filters/) for the tremendous help in the creation of my code

Monte Carlo Localization can be highly useful for improving the robustness and accuracy of the robot’s position tracking, especially in environments with uncertainty or complex features. Since VEX robots often operate in known environments, MCL allows the robot to maintain accurate localization by sampling multiple points of its position and refining them based on data from the distance sensors.

MAKE SOME DISCLAIMERS
1. OUR MAP BEING ROTATED WITH FRONT BEING +Y
2. CODE BEING WEIRDLY NAMED, CRY ABOUT IT
3. NOT FULLY INDEPTH IF YOU WANT TO FULLY UNDERSTAND GO READ MY FULL PAPER ON IT

### Particles

The basis of MCL is the particle filter, where a “particle” is just a representation of the state of the robot. Since we know the state of the robot (or the robot variable we care about) are ```X, Y, and Theta```, we made a **Struct** to be a lightweight object to use for our particles.

![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)

### Sensors

For MCL, the sensors we use are distance sensors. We created a custom class specifically for the MCL Distance Sensors to be able to only need to measure once every loop, adjust for the offsets from the sensors, and keep everything in one place.

![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)

## Field

For the map of the field, we created our own Field class which holds all the information we need about the field to be able to understand our map.

### Game Objects (Mobile Goals)

* The ```Goal``` struct represents a mobile goal, which has:
  * ```Position```: A ```Point``` representing the goal's position on the field.
  * ```RadiusSquared```: The squared radius of the goal for fast distance calculations.
  * A constructor that initializes the goal’s position and stores the squared radius.

### Constructor

* The ```Field``` constructor initializes the field with a predefined set of four mobile goals, located at ```(±24, 0)``` and ```(0, ±24)```, each with a radius of ```2``` inches.

### Field Properties and Functions

* ```GetSize()```: Returns the full length/width of the field by doubling ```HalfSize```.
* ```AddGoal(Point P, float R)```: Adds a new mobile goal to the ```Goals``` vector with a given position and radius.

### Sensor Distance Measurement

* ```get_sensor_distance(Particle& p, const MCLDistanceSensor& Sensor)```:
  * **Sensor Position Calculation:** Computes the sensor's position based on the particle's position and sensor offset.
  * **Step Vector Calculation:** Rotates the particle’s motion vector based on the sensor’s direction.
  * **Goal Intersection Check:**
    * Determines if the sensor’s path intersects any goal using vector projections.
    * If the sensor’s heading points toward the goal and the perpendicular distance is within the goal’s radius, calculate the intersection distance.
  * **Wall Intersection Calculation:**
    * If no goal is hit, calculate the distance to the nearest vertical or horizontal wall.
    * Uses trigonometry to determine the intersection point.

### Field Constants

* ```HalfSize```: Represents half the field’s size (```140.875 / 2```), used for wall calculations.
* ```direction_to_sine``` and ```direction_to_cosine```: Lookup tables for sensor direction calculations.

As well as the ```HalfSize``` of the field as for our purposes the middle of the field is (0, 0) so the furthest distance possible is +- 72 inches. This class is designed for the MCL, where the robot’s sensors need to detect mobile goals and walls. By efficiently calculating distances to objects, it helps navigation

![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)

The most important part of this field class is the function

``` cpp
get_sensor_distance(Particle& p, const MCLDistanceSensor& Sensor)
```

This function takes in a particle, and a corresponding distance sensor object to be able to calculate the expected value of that sensor to be used in the **update step.**

It uses some clever math to be able to do so as described here

Let $d$ be the distance to the wall. Let $\theta$ be the sensor heading. We can take advantage of the square shape of the field to determine the distance to the wall.

For a vertical wall:

$x+d\cos{\theta}=x_{wall}\rightarrow d=(x_{wall} - x)/\cos{\theta}$

Using that, and Intersection occurs if |y + d sin(θ)| < max_y.

$|y+ sin{\theta}| < y_{max}$

Horizontal walls are similar.



---

# Initialization

The ```StartMCL``` function initializes the Monte Carlo Localization (MCL) process by creating and distributing particles around the robot's starting position. It sets up the initial belief of the robot’s position by adding noise to the given ```(x_, y_, theta_)``` values. Here’s a breakdown of what each part does. This function generates a set of particles that approximate the initial position of the robot. Each particle represents a possible state of the robot with small variations in position and orientation.

# Main Algorithm

## Predict

The predict section of the MCL is responsible for predicting the next state of each particle based on the robot's motion model. It propagates the particles forward according to the robot’s estimated velocity while incorporating random noise to account for uncertainty.

1. **Get the Robot’s Average Velocity**
   * ```Velo = getAvgVelocity() * VeloScale;```
   * Computes the average velocity of the robot and scales it with ```VeloScale``` to ensure the motion model is properly adjusted. NEED TO DESCRIBE HOW VELO SCALE IS CALCULATED
2. **Create a Normal Distribution for Noise NEED TO TALK ABOUT SCALED NOISE DISRTUBUTION**
    * ```normal_distribution<double> dist_pos(0, Velo / 4)```;
    * This distribution models the uncertainty in motion by generating small random deviations in the particle positions.
3. **Calculate the Correct Robot’s Angle (```Theta```)**
    * ```const double theta_ = Angle.rotation(degrees) * toRad + start_theta;```
    * Ensures that the robot’s angle is in radians (Interial converted from Degrees) and is algnied with the correct starting angle.
    * ```const double rotated_theta = M_PI_2 - theta_;```
    * Adjusts the angle to align with the coordinate system used for motion calculations.
4. **Precompute Sin and Cos Values for Efficiency**
    * ```const float cos_theta = cosf(rotated_theta);```
    * ```const float sin_theta = sinf(rotated_theta);```
    * Computes the cosine and sine of the adjusted angle in advance to speed up calculations.
5. **Compute the Velocity Components**
    * ```const float velo_cos = Velo * cos_theta```
    * ```const float velo_sin = Velo * sin_theta;```
    * Breaks the velocity into ```x``` and ```y``` components for applying movement.
6. **Loop Through Each Particle to Update Position**
    * ```for (auto& p : Particles)```
    * Iterates over all particles and applies the motion model.
7. **Update Particle Orientation**
    * ```p.theta = theta_;```
    * Ensures each particle has the updated orientation.
8. **Set the Step Vector (Direction of Movement)**
    * ```p.step = Point(cos_theta, sin_theta);```
    * Stores the direction in which the particle is moving.
9. **Apply Motion Update with Noise**
    * ```p.x += velo_cos + dist_pos(Random);```
    * ```p.y += velo_sin + dist_pos(Random);```
    * Moves the particle based on the computed velocity while adding random noise to simulate real-world movement uncertainty.
10. **Clamp Particle Position Within Field Boundaries**
    * ```p.x = clamp(p.x, -field_.HalfSize, field_.HalfSize);```
    * ```p.y = clamp(p.y, -field_.HalfSize, field_.HalfSize);```
    * Ensures that particles stay within the field limits, preventing them from drifting outside the simulation area.

## Update

The update section of the MCL performs the weight update step, where each particle's likelihood is adjusted based on sensor measurements. The weight of each particle is computed using the **Gaussian (normal) distribution equation**, ensuring that particles more consistent with sensor readings are given higher weights.

### Breakdown of Key Steps:

1. **Measure Sensor Readings**
    - Calls the `Measure` function for each sensor, retrieving distance measurements.
2. **Filter Out Invalid Sensor Readings**
    - Sensors with valid measurements (`> -1`) are stored in `activeSensors` for further processing.
3. **Initialize Weight Sum**
    - `weights_sum = 0;`
    - Prepares for weight normalization by resetting the sum of all particle weights.
4. **Skip Weight Update If No Active Sensors**
    - `if (!activeSensors.empty())`
    - If no valid sensor data is available, the weight update step is skipped.
5. **Compute Particle Weights Based on Sensor Readings**
    - **Iterate Through Each Particle**
        - Initialize weight (`wt = 1.0`) for each particle.
    - **Predict Measurement for Each Active Sensor**
        - Uses the environment model (`field_`) to estimate what the sensor would detect from the particle’s position.
    - **Discard Particles With Invalid Predictions**
        - If the predicted sensor reading is invalid (negative distance), the particle is given a weight of `0`.
    - **Calculate the Error Between Prediction and Measurement**
        - Computes the difference between the predicted and actual sensor measurement.
    - **Update Particle Weight Using the Gaussian Distribution Equation**
        
        The Gaussian (normal) distribution is applied to model uncertainty in sensor readings:
        
        ![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)
        
        - This is implemented in the code as:
            
            ``` cpp
            wt *= exp((Deviation * Deviation) * inv_varience) * inv_base;
            ```
            
        - Where:
            - `Deviation` is the difference between predicted and actual measurement.
            - `inv_varience` is the inverse of twice the variance (With a standard deviation of 2 inches).
            - `inv_base` is the normalization factor.
6. **Update Total Weight Sum and Assign Weights to Particles**
    - Each particle’s computed weight is stored, and `weights_sum` keeps track of the total sum of all weights.
7. **Normalize Weights to Ensure a Probability Distribution**
    - **If Total Weight Is Too Small, Reset All Weights**
        - If all weights are too small (e.g., due to numerical underflow), set each particle’s weight to an equal probability.
    - **Otherwise, Normalize All Weights**
        - Ensures that the sum of all weights equals `1.0`, maintaining a proper probability distribution.

![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)

## Resampling

Resampling is crucial for Monte Carlo Localization for several reasons:

Without resampling, most particles would eventually have negligible weights, effectively wasting computational resources on particles that contribute little to the position estimate. Resampling also allows the algorithm to focus particles in regions of the state space with high probability, improving accuracy. As the robot moves and receives sensor data, resampling helps the particle distribution adapt to new information by replacing unlikely particles with more likely ones. **Resampling ensures that particles with higher weights (i.e., those that better match sensor measurements) are more likely to be chosen for the next iteration, while lower-weight particles are removed.**

For our implementation, we’re resampling using stratified resampling.
![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)
![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)

The stratified approach used in this code has advantages over simpler methods:
**Better Coverage:** By ensuring samples are drawn from each interval, stratified resampling maintains better diversity in the particle set.
**Reduced Variance:** The variance of the resampling process is reduced compared to purely random sampling.
**Preservation of Distribution Shape:** The approach better preserves the shape of the original distribution while still focusing on high-probability regions.
**Efficiency:** The implementation uses a binary search for efficient sampling, making it computationally effective.

This approach helps the MCL algorithm balance between exploration (maintaining diverse particles) and exploitation (focusing on likely positions), which is essential for robust robot localization.

![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)













## Update Pose

This section updates the estimated pose (X,Y,θ) of the robot based on the weighted average of all particles. It ensures that the final estimated position and orientation reflect the most probable location of the robot according to the particle filter.

![image.png](attachment:64e81b9e-d72e-4e7d-bc0b-6d014966a7b5:image.png)

## References


> 1.  “Interactive Robotics Algorithms.” Utexas.edu, 2024, amrl.cs.utexas.edu/interactive-particle-filters/.

> 2. Mantelli, Mathias. “Particle Filter Part 1 — Introduction - Mathias Mantelli - Medium.” Medium, 5 Aug. 2023, medium.com/@mathiasmantelli/particle-filter-part-1-introduction-fb6954bc12ec.

> 3. “Particle Filter | Probabilistic Robotics.” Gitbook.io, 18 July 2019, calvinfeng.gitbook.io/probabilistic-robotics/basics/nonparametric-filters/02-particle-filter.

> 4. rlabbe. “Kalman-And-Bayesian-Filters-In-Python/12-Particle-Filters.ipynb at Master · Rlabbe/Kalman-And-Bayesian-Filters-In-Python.” GitHub, 2024, github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb. 

> 5. Wikipedia Contributors. “Monte Carlo Localization.” Wikipedia, Wikimedia Foundation, 4 July 2019, en.wikipedia.org/wiki/Monte_Carlo_localization.

> 6. Maxx Wilson | VEXU GHOST


