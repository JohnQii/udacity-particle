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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

/**
 * @brief ParticleFilter::init
 * @param x
 * @param y
 * @param theta
 * @param std
 * Initialize all particles
 */
void ParticleFilter::init(double x, double y, double theta, double std[], double yaw_eps) {
  if(is_initialized)
  {
    std::cout<<"Have Initialized before "<<std::endl;
    return;
  }
  yaw_eps_ = yaw_eps;
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
  num_particles = 10;
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  // Add random Gaussian noise to each particle.

  //init it!
  default_random_engine gen;
  for (int i = 0; i < num_particles; i++)
  {
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;

    particles.push_back(particle);
  }
  is_initialized = true;
  std::cout<<"Initialized! "<<std::endl;
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/
  default_random_engine gen;
  //normal distributions for noise
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);
  //predict the next state
  for (int i = 0; i < num_particles; i++)
  {
    double theta = particles[i].theta;

    if (fabs(yaw_rate) < yaw_eps_) // When yaw is not changing.
    {
      particles[i].x += velocity * delta_t * cos( theta );
      particles[i].y += velocity * delta_t * sin( theta );
//      particles[i].theta = particles[i].theta;//continue to be the same.
    }
    else
    {
      particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
      particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
      particles[i].theta += yaw_rate * delta_t;
    }

    // add noise.
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.

  for (uint i = 0; i < observations.size(); ++i)
  {
    double lowest_dist = numeric_limits<double>::max();

    int map_id = -1;
    double obs_x = observations[i].x;
    double obs_y = observations[i].y;

    for (uint j = 0; j < predicted.size(); j++)
    {
      double predicted_x = predicted[j].x;
      double predicted_y = predicted[j].y;
      int predicted_id = predicted[j].id;
      double cur_dist = dist(obs_x, obs_y, predicted_x, predicted_y);

      if (cur_dist < lowest_dist)
      {
        lowest_dist = cur_dist;
        map_id = predicted_id;
      }
    }
    observations[i].id = map_id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html
  weights.clear();
  double normalizer = (1.0/(2.0 * M_PI * std_landmark[0] * std_landmark[1]));
  double weight_normalizer = 0.0;
  for (int i = 0; i < num_particles; i++)
  {
    //0. get particles
    double par_x = particles[i].x;
    double par_y = particles[i].y;
    double par_theta = particles[i].theta;

    //1. transform observation coordinates.
    std::vector<LandmarkObs> transformed_observations;
    for(unsigned int j = 0; j < observations.size(); j++)
    {
      double x = cos(par_theta)*observations[j].x - sin(par_theta)*observations[j].y + par_x;
      double y = sin(par_theta)*observations[j].x + cos(par_theta)*observations[j].y + par_y;
      transformed_observations.push_back(LandmarkObs{ observations[j].id, x, y });
    }

    //2. find the map landmarks which are in range.
    std::vector<LandmarkObs> ranged_landmarks;
    for (uint j = 0; j < map_landmarks.landmark_list.size(); j++)
    {
      Map::single_landmark_s current_landmark = map_landmarks.landmark_list[j];
      if ((fabs((par_x - current_landmark.x_f)) <= sensor_range) && (fabs((par_y - current_landmark.y_f)) <= sensor_range))
      {
        ranged_landmarks.push_back(LandmarkObs {current_landmark.id_i, current_landmark.x_f, current_landmark.y_f});
      }
    }

    //3. dataAssociation
    dataAssociation(ranged_landmarks, transformed_observations);

    //4. Calculate weights.
    particles[i].weight = 1.0;

    for(uint j = 0; j < transformed_observations.size(); ++j)
    {
      double trans_x = transformed_observations[j].x;
      double trans_y = transformed_observations[j].y;
      double trans_id = transformed_observations[j].id;
      double multi_prob = 1.0;

      for (uint k = 0; k < ranged_landmarks.size(); ++k) {
        double landmark_x = ranged_landmarks[k].x;
        double landmark_y = ranged_landmarks[k].y;
        double landmark_id = ranged_landmarks[k].id;

        if (trans_id == landmark_id) {
          multi_prob = normalizer * exp(-1.0 * ((pow((trans_x - landmark_x), 2)/(2.0 * std_landmark[0] * std_landmark[0]))
                       + (pow((trans_y - landmark_y), 2)/(2.0 * std_landmark[1] * std_landmark[1]))));
          particles[i].weight *= multi_prob;
        }
      }
    }
    weight_normalizer += particles[i].weight;
  }
  //normalised
  for (uint i = 0; i < particles.size(); i++)
  {
    particles[i].weight /= weight_normalizer;
    weights.push_back(particles[i].weight);
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  vector<Particle> resampled_particles;

  // Create a generator to be used for generating random particle index and beta value
  default_random_engine gen;

  //Generate random particle index
  uniform_int_distribution<int> particle_index(0, num_particles - 1);

  int current_index = particle_index(gen);

  double beta = 0.0;

  double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());

  for (int i = 0; i < particles.size(); i++) {
    uniform_real_distribution<double> random_weight(0.0, max_weight_2);
    beta += random_weight(gen);

    while (beta > weights[current_index]) {
      beta -= weights[current_index];
      current_index = (current_index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[current_index]);
  }
  particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                         const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
