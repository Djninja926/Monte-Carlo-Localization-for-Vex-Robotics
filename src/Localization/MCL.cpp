// Default
#include "vex.h"
#include "cmath"
#include "Localization/MCL.h"

using namespace std;

namespace MCL {
  // Position Variables
  double X = 0, Y = 0, theta = 0;

  // Random Number Generator
  mt19937 Random(random_device {} ());

  /* Prediction */
  double Velo = 0;

  /* Update */
    double Deviation;
    double weights_sum;
    double start_theta;

  /* Vectors */
    // Vectors of Particles
    vector<Particle> Particles;
    
    // Vector of Sensors
    vector<MCLDistanceSensor> Sensors = {
      MCLDistanceSensor(FrontDis, Point(-3.75, -3.75), FRONT),
      MCLDistanceSensor(RightDis, Point(1.5, -5.25), RIGHT),
      MCLDistanceSensor(LeftDis, Point(1.5, 5.25), LEFT),
      MCLDistanceSensor(BackDis, Point(-3.25, -2.75), BACK), 
    };

    // The Field
    Field field_;

  /* Vectors */    
    vector<MCLDistanceSensor> activeSensors;

  /* Helper Functions */
    double getAvgVelocity(void);

  /* Start Function */
    void StartMCL(double x_, double y_, double theta_);

  /* Main Loop */
  void MonteCarlo(void);


  void FrontTurn(double TurnTargetX, double TurnTargetY, double HighestPower, double MiniPower, bool TurnAccuracy, vex::brakeType braking) {
    // Get the angle from the rotated graph
    double TargetAngle = atan2((TurnTargetX - MCL::X) * -1, TurnTargetY - MCL::Y) * -1;
    // Wrap the target heading around 360 (0 - 360)
    if (TargetAngle < 0) TargetAngle += (2 * M_PI);
    // Use the angle to get the global target heading
    double Amount = (TargetAngle - AngleWrap(theta));
    // Wrap the Turning angle around 180
    if (Amount > M_PI) {
      Amount -= 2 * M_PI;
    } else if (Amount < -M_PI) {
      Amount += 2 * M_PI;
    }
    // Input into PID
    // std::cout << ToDegrees(Amount) << "\n";
    TurnPID::TurnPidOn(ToDegrees(Amount), HighestPower, MiniPower, TurnAccuracy, braking);
  }

  void BackTurn(double TurnTargetX, double TurnTargetY, double HighestPower, double MiniPower, bool TurnAccuracy, vex::brakeType braking) {
    // Get the angle from the rotated graph
    double TargetAngle = atan2(TurnTargetX - MCL::X, -1 * (TurnTargetY - MCL::Y)) * -1;
    // Wrap the target heading around 360 (0 - 360)
    if (TargetAngle < 0) TargetAngle += (2 * M_PI);
    // Use the angle to get the global target heading
    double Amount = (TargetAngle - AngleWrap(theta));
    // Wrap the Turning angle around 180
    if (Amount > M_PI) {
      Amount -= 2 * M_PI;
    } else if (Amount < -M_PI) {
      Amount += 2 * M_PI;
    }
    // Input into PID
    // std::cout << ToDegrees(Amount) << "\n";
    TurnPID::TurnPidOn(ToDegrees(Amount), HighestPower, MiniPower, TurnAccuracy, braking);
  }
}

/* Set the number of particles. Initialize all particles to first position (based on estimates of
x, y, theta and their uncertainties from GPS) and all weights to 1. Add random Gaussian noise to each particle.
*/

// Initialize particles at the start position of the robot
void MCL::StartMCL(double x_, double y_, double theta_) {
  // Clearing All Vectors
  Particles.clear();

  // Resize Particle Vector (reserve memory to avoid reallocations)
  Particles.reserve(num_particles);

  // Uniform Distribution in Range +-
  uniform_real_distribution<double> start_dist_x(-start_std_pos[0], start_std_pos[0]);
  uniform_real_distribution<double> start_dist_y(-start_std_pos[1], start_std_pos[1]);
  uniform_real_distribution<double> start_dist_theta(-start_std_pos[2], start_std_pos[2]);

  // // Resize Particle Vector
  // Particles.resize(num_particles);

  start_theta = toRad * theta_;

  /* Randomize around Sstarting Position */
  for (int i = 0; i < num_particles; ++i) {
    // Setting all the Values
    Particles[i].x = x_ + start_dist_x(Random);
    Particles[i].y = y_ + start_dist_y(Random);
    Particles[i].theta = ToRadians(theta_) + start_dist_theta(Random);
    Particles[i].weight = 1.0;
  }
}

// Main Loop
void MCL::MonteCarlo(void) {
  timer MCLTimer;
  while (true) {
    MCLTimer.reset();
  /* Prediction */
    // Getting the Average Velocity of the Robot
    Velo = getAvgVelocity() * VeloScale;

    // Make the Normal Distribution
    normal_distribution<double> dist_pos(0, Velo / 4);

    // Normalized Theta to [-π, π]
    const double theta_ = Angle.rotation(degrees) * toRad + start_theta;
    const double rotated_theta = M_PI_2 - theta_;

    // Pre Compute sin and cos
    const float cos_theta = cosf(rotated_theta);
    const float sin_theta = sinf(rotated_theta);

    // Pre Compute X and Y Velocity
    const float velo_cos = Velo * cos_theta;
    const float velo_sin = Velo * sin_theta;

    // Loop through all the Particles
    for (auto& p : Particles) {
      // Theta - Normalized to [-π, π]
      p.theta = theta_;

      // Setting the Step Vector
      p.step = Point(cos_theta, sin_theta);

      // Changing the State Vars
      p.x += velo_cos + dist_pos(Random);
      p.y += velo_sin + dist_pos(Random);

      // Clamp to Inside the Field
      p.x = clamp(p.x, -field_.HalfSize, field_.HalfSize);
      p.y = clamp(p.y, -field_.HalfSize, field_.HalfSize);
    }

  /* Update */
    // Measure Each Sensor
    for (auto& sensor : Sensors) {
      sensor.Measure();
    }

    // Clear from Previous Loop
    activeSensors.clear();

    for (auto& sensor : Sensors) {
      if (sensor.measurement > -1) {
        activeSensors.push_back(sensor);
      }
    }

    // Reset the Weight Sum
    weights_sum = 0;

    // Skip weight update if no active sensors
    if (!activeSensors.empty()) {
      for (auto& P : Particles) {
        // New Weight
        double wt = 1.0;
        /* Find the Predicted Distance */
        for (auto& sensor : activeSensors) {
          // Skip if Out of Range
          if (sensor.measurement == -1) continue;
          // Predicted Measurement from Particle
          const double predicted = field_.get_sensor_distance(P, sensor);
          // Remove Particle if Expected is bad
          if (predicted < 0) {
            wt = 0;
            break;
          }

          // Calculate the Error
          Deviation = (predicted - sensor.measurement);

          // Update Weights using Gaussian Distribution Equation: 1/2e^    1 / (σ√(2π))) * e^((x-μ)^2 / (-2σ^2))
          wt *= exp((Deviation * Deviation) * inv_varience) * inv_base;
        }
        // Update the Weight and Sum of Weights
        weights_sum += wt;
        P.weight = wt;
      }
    }

    /* Normalize Weights */
    if (weights_sum < MIN_WEIGHT) {
      const double Weight = inv_num_particles;
      for (auto &p : Particles) {
        p.weight = Weight;
      }
    } else {
      const double inv = 1.0 / weights_sum;
      for (auto &p : Particles) {
        p.weight *= inv;
      }
    }

  /* Resample Using Stratified Resampling */
    vector<Particle> Resampled(num_particles);
    
    // Cumulative Distribution Function Vector
    vector<double> CDF(num_particles);

    // Compute Cumulative Distribution Function (CDF)
    CDF[0] = Particles[0].weight;
    for (int i = 1; i < num_particles; ++i) {
      CDF[i] = CDF[i - 1] + Particles[i].weight;
    }

    // Random Distance Generator in Range (0, 1.0 / N]
    uniform_real_distribution<double> dist(0, inv_num_particles);

    // Generate Random Points in Each Interval
    for (int i = 0; i < num_particles; ++i) {
      const double Point = i * (inv_num_particles) + dist(Random);
      // Binary search for faster index finding
      const auto it = lower_bound(CDF.begin(), CDF.end(), Point);
      const int index = std::distance(CDF.begin(), it); 
      // Push Back New Particle
      Resampled[i] = Particles[index];
    }

    // Updating the Particles with the Resampled Ones
    Particles.swap(Resampled);
  /*
    vector<Particle> Resampled;
    Resampled.resize(num_particles);

    // Cumulative Distribution Function Vector
    vector<double> CDF(num_particles);

    // Compute Cumulative Distribution Function (CDF)
    CDF[0] = Particles[0].weight;
    for (int i = 1; i < num_particles; ++i) {
      CDF[i] = CDF[i - 1] + Particles[i].weight;
    }

    // Generate a single random starting point
    uniform_real_distribution<double> dist(0, inv_num_particles);

    // Calculate Start and Step
    const double start = dist(Random);

    // Use systematic sampling with fixed step size
    for (int i = 0; i < num_particles; ++i) {
      const double Point = start + i * inv_num_particles;
      const auto it = lower_bound(CDF.begin(), CDF.end(), Point);
      const int index = std::distance(CDF.begin(), it);
      Resampled[i] = Particles[index];
    }

    // Updating the Particles with the Resampled Ones
    Particles.swap(Resampled);
  */
    
  /* Update Pose */
    double new_x = 0.0, new_y = 0.0, new_theta = 0.0, total_weight = 0.0;

    // Average the Position
    for (int i = 0; i < num_particles; ++i) {
      const auto& p = Particles[i];
      const double wt = p.weight;
      new_x += p.x * wt;
      new_y += p.y * wt;
      new_theta += p.theta * wt;
      total_weight += wt;
    }

    // Normalize by total weight
    if (total_weight > 0) {
      const double inv_weight = 1.0 / total_weight;
      new_x *= inv_weight;
      new_y *= inv_weight;
      new_theta  *= inv_weight;
    }

    // Clamp it from Changing too much
    new_x = clamp(new_x, X - MAXSTEP, X + MAXSTEP);
    new_y = clamp(new_y, Y - MAXSTEP, Y + MAXSTEP);

    // Setting the Estimate
    X = new_x, Y = new_y, theta = new_theta;

    // cout << MCLTimer.time() << endl;
    // cout << "X: " << X << "      Y: " << Y << endl;
  }
}

double MCL::getAvgVelocity(void) noexcept {
  const double V = (L1.velocity(rpm) + L2.velocity(rpm) + L3.velocity(rpm) + R1.velocity(rpm) + R2.velocity(rpm) + R3.velocity(rpm)) * (1.0 / 6.0);
  return V;
}

// Define the constexpr arrays
constexpr float Field::direction_to_sine[];
constexpr float Field::direction_to_cosine[];