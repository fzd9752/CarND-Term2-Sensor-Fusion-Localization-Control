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
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "map.h"

using namespace std;



void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  //cout << "Start Initialize..." << endl;
  default_random_engine gen;
  // Add Guasian noise to GPS Initialize data
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  // Set particles number
  num_particles = 100;
  // Init particles
  for (unsigned i=0; i < num_particles; i++)
  {
    Particle p;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
    weights.push_back(p.weight);
    //cout << "p.weight: " << " " << particles[i].weight << endl;
    //cout << "weights: " << " " << weights[i];
  }
  is_initialized = true;
  //cout << "Initialized." << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  //cout << "Start Predict..." << endl;
  default_random_engine gen;

  for (unsigned i=0; i < num_particles; i++)
  {
    Particle p = particles[i];
    //cout << "p_x: " << particles[i].x << endl;
    //cout << "p_weight: " << particles[i].weight << endl;

    double v_yaw = velocity/yaw_rate;
    double delta_yaw = yaw_rate*delta_t;
    // Update particels
    if (fabs(yaw_rate) > 0.0001)
    { // yaw_rate is not zero
      p.x += v_yaw*(sin(p.theta + delta_yaw) - sin(p.theta));
      p.y += v_yaw*(cos(p.theta) - cos(p.theta + delta_yaw));
      p.theta += delta_yaw;
    }
    else
    { // yaw_rate is zero
      p.x += velocity*cos(p.theta)*delta_t;
      p.y += velocity*sin(p.theta)*delta_t;
    }
    // Generate Guasian noise distribution
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    // Add Gaussian noise
    particles[i].x = p.x + dist_x(gen);
    particles[i].y = p.y + dist_y(gen);
    particles[i].theta = p.theta + dist_theta(gen);
    //cout << "weight after predicted: " << particles[i].weight << endl;
  }
  //cout << "Predicted." << endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

  //cout << "Start update..." << endl;
  double x_deno = 2.0*std_landmark[0]*std_landmark[0];
  double y_deno = 2.0*std_landmark[1]*std_landmark[1];
  double deno = 2.0*M_PI*std_landmark[0]*std_landmark[1];

  weights.clear();

  for (unsigned i=0; i < num_particles; i++)
  {
    Particle p = particles[i];
    //cout << "Particle " << i << ": " << endl;
    //cout << "weight: " << p.weight << endl;
    // Step 0 Filter over-range map_landmarks
    vector<LandmarkObs> predicted;
    //cout << "predicted size: " << predicted.size() << '\n';
    for (unsigned int l = 0; l < map_landmarks.landmark_list.size(); l++)
    {
      Map::single_landmark_s m = map_landmarks.landmark_list[l];
      //cout << m.x_f << endl;
      //cout << m.y_f << endl;
      double dist_range = dist(p.x, p.y, m.x_f, m.y_f);
      //cout << dist_range << endl;
      if (dist_range < sensor_range)
      {
        LandmarkObs map_landmark;
        map_landmark.id = m.id_i;
        map_landmark.x = m.x_f;
        map_landmark.y = m.y_f;
        predicted.push_back(map_landmark);
      }
    }
    //cout << "predicted size: " << predicted.size() << '\n';

    double w = 1.0;
    for (unsigned n=0; n < observations.size(); n++)
    {
      // Step 1 transform observations to map coordinates
      //cout << "Start Transform..." << endl;
      LandmarkObs tob;
      tob.x = p.x + observations[n].x*cos(p.theta) - observations[n].y*sin(p.theta);
      tob.y = p.y + observations[n].x*sin(p.theta) + observations[n].y*cos(p.theta);
      tob.id = observations[n].id;
      //cout << "Transformed." << endl;
      //cout << "x: " << tob.x << "  y: " << tob.y << endl;
      // Step 2 Association
      //cout << "Start Associate..." << endl;
      double min_dist = 100000.0;
      double ind=0;
      for (unsigned p=0; p < predicted.size(); p++)
      {
        double distance = dist(tob.x, tob.y, predicted[p].x, predicted[p].y);
        //cout << "distance from tob: " << distance << endl;
        if (min_dist > distance)
        {
          min_dist = distance;
          ind = p;
          //cout << "min_dist: " << min_dist << endl;
          //cout << "ind: " << ind << endl;
          //cout << "predicted_i: " << predicted[ind].id << endl;
        }
      }
      LandmarkObs aob = predicted[ind];
      //cout << "observations " << endl << "id: " << tob.id << "  "
      //<< "x: " << tob.x << "  " << "y: " << tob.y << endl;
      //cout << "associations " << endl << "id: " << aob.id << "  "
      //<< "x: " << aob.x << "  " << "y: " << aob.y << endl;
      //cout << "Associated." << endl;
      // Step 3 Calculate Mult-variate Gaussian distribution and new weight
      //cout << "Start Cacluate Mult-variate..." << endl;
      double x_part = -(tob.x - aob.x)*(tob.x - aob.x)/x_deno;
      double y_part = -(tob.y - aob.y)*(tob.y - aob.y)/y_deno;
      double prob = exp(x_part + y_part)/deno;
      if (prob > 0)
      {
        w *= prob;
      }
      //cout << "prob: " << " " << prob << endl;
      //cout << "update w: " << w << endl;
    }
    particles[i].weight = w;
    weights.push_back(w);
    //cout << "w: " << w << endl;
    //cout << "updated weight: " << particles[i].weight << "  " << w << endl;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  random_device rd;
  mt19937 gen(rd());
  discrete_distribution<> d(weights.begin(), weights.end());

  vector<Particle> particles_new;
  for (unsigned i=0; i<num_particles;i++){
    particles_new.push_back(particles[d(gen)]);
  }
  particles = particles_new;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
