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
#include <cmath>
#include <limits>

#include "particle_filter.h"

const std::size_t kNumberOfParticles = 100U;
const double kZeroTolerance = 1.0E-3;

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
    std::default_random_engine gen;
    std::normal_distribution<double> norm_x    (0.0, std_pos[0U]);
    std::normal_distribution<double> norm_y    (0.0, std_pos[1U]);
    std::normal_distribution<double> norm_theta(0.0, std_pos[2U]);

    for (Particle &p : particles)
    {
        // Predict
        if (std::fabs(yaw_rate) < kZeroTolerance)
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

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> /*predicted*/,
                                     std::vector<LandmarkObs>& /*observations*/)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    // Function not used due to unclear documentation

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        std::vector<LandmarkObs> observations, Map map_landmarks)
{
    double weight_sum = 0.0;

    const double std_x = std_landmark[0U];
    const double std_y = std_landmark[1U];

    // Loop over particles
    for (Particle& p : particles)
    {
        double total_prob = 1.0;  // To multiply probabilities later

        const double c_theta = std::cos(p.theta);
        const double s_theta = std::sin(p.theta);

        // Loop over measurements
        for (LandmarkObs& z : observations)
        {
            // Transform measurements from car frame to particle frame
            const double z_x_car = z.x;
            const double z_y_car = z.y;

            const double z_x_p = p.x + z_x_car*c_theta - z_y_car*s_theta;
            const double z_y_p = p.y + z_x_car*s_theta + z_y_car*c_theta;

            // Perform nearest neighbor data association
            double d_min = std::numeric_limits<double>::max();
            std::size_t id_min = map_landmarks.landmark_list.size();

            for (std::size_t i = 0U; i < map_landmarks.landmark_list.size(); ++i)
            {
                const Map::single_landmark_s& m_i  = map_landmarks.landmark_list[i];

                const double d_p_to_landmark = dist(p.x, p.y, m_i.x_f, m_i.y_f);

                if (d_p_to_landmark < sensor_range)
                {
                    const double d_z_to_landmark =
                            dist(z_x_p, z_y_p, m_i.x_f, m_i.y_f);

                    if (d_z_to_landmark < d_min)
                    {
                        d_min = d_z_to_landmark;
                        id_min = i;
                    }
                }
            }

            if (id_min == map_landmarks.landmark_list.size())
            {
                // Could not find associated landmark within sensor_range
                continue;
            }

            // Compute probability and update weight
            const double m_x = map_landmarks.landmark_list[id_min].x_f;
            const double m_y = map_landmarks.landmark_list[id_min].y_f;

            const double k = 1.0 / (2.0 * M_PI * std_x * std_y);
            const double a = (z_x_p - m_x) * (z_x_p - m_x) / (std_x * std_x);
            const double b = (z_y_p - m_y) * (z_y_p - m_y) / (std_y * std_y);

            const double prob_z = k * std::exp(-0.5 * (a + b));

            total_prob *= prob_z;
        }

        // Assign weight = 0.0 if no observations were associated to landmarks
        p.weight = (total_prob != 1.0 ? total_prob : 0.0);
        weight_sum += p.weight;
    }

    // Normalize weights
    for (Particle& p : particles)
    {
        p.weight /= weight_sum;
    }
}

void ParticleFilter::resample()
{
    // Store the weights in a vector for convenience
    for (std::size_t i = 0U; i < particles.size(); ++i)
    {
        weights[i] = particles[i].weight;
    }

    // Resample according to weights
    std::vector<Particle> particles_original(particles);
    std::default_random_engine gen;
    std::discrete_distribution<std::size_t> d(weights.begin(), weights.end());

    for (Particle& p : particles)
    {
        p = particles_original[d(gen)];
    }
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
