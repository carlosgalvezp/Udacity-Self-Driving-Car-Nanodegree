/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>

#include "particle_filter.h"

const std::size_t kNumberOfParticles = 100U;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    // Allocate memory for particles and weights
    particles.resize(kNumberOfParticles);
    weights.resize(kNumberOfParticles);

    // Randomly initialize based on initial GPS position and std
    std::default_random_engine gen;
    std::normal_distribution<double> norm_x    (x,     std[0U]);
    std::normal_distribution<double> norm_y    (y,     std[1U]);
    std::normal_distribution<double> norm_theta(theta, std[2U]);

    int id = 0;
    for (Particle &p : particles)
    {
        p.id = id;
        p.x = norm_x(gen);
        p.y = norm_y(gen);
        p.theta = norm_theta(gen);
        p.weight = 1.0 / static_cast<double>(kNumberOfParticles);

        ++id;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    std::default_random_engine gen;
    std::normal_distribution<double> norm_x    (0.0, std_pos[0U]);
    std::normal_distribution<double> norm_y    (0.0, std_pos[1U]);
    std::normal_distribution<double> norm_theta(0.0, std_pos[2U]);

    for (Particle &p : particles)
    {
        // Predict
        if (std::fabs(yaw_rate) < 1.0E-3)
        {
            p.x += velocity * delta_t * std::cos(p.theta);
            p.y += velocity * delta_t * std::sin(p.theta);
            p.theta = p.theta;
        }
        else
        {
            const double theta_new = p.theta + yaw_rate * delta_t;

            p.x += (velocity/yaw_rate) * ( std::sin(theta_new) - std::sin(p.theta));
            p.y += (velocity/yaw_rate) * (-std::cos(theta_new) + std::cos(p.theta));
            p.theta = p.theta + yaw_rate * delta_t;
        }

        // Add noise
        p.x     += norm_x(gen);
        p.y     += norm_y(gen);
        p.theta += norm_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        std::vector<LandmarkObs> observations, Map map_landmarks)
{
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
