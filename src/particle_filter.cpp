#include <iostream>
#include <cmath>
#include <random>
#include <numeric>
#include <algorithm>

#include "particle_filter/particle_filter.h"


ParticleFilter::ParticleFilter(int particles_num, Eigen::VectorXd initState, Eigen::VectorXd initCovariance)
{
    n_ = particles_num;
    state_ = initState;
    w_.resize(n_);
    w_.setZero();
    particles_.resize(3, n_);
    particles_.setZero();
    stateCovariance_ = initCovariance;

    double sigma_x, sigma_y, sigma_th;
    sigma_x = initCovariance(0);
    sigma_y = initCovariance(1);
    sigma_th = initCovariance(2);

    std::normal_distribution<double> dist_x(state_(0), sigma_x);
    std::normal_distribution<double> dist_y(state_(1), sigma_y);
    std::normal_distribution<double> dist_th(state_(2), sigma_th);

    std::default_random_engine gen;

    // Create particles
    for (int i=0; i<n_; i++)
    {
        particles_(0,i) = dist_x(gen);
        particles_(1,i) = dist_y(gen);
        particles_(2,i) = dist_th(gen);
        w_(i) = 1.0;
    }

    initialized_ = true;
}

ParticleFilter::~ParticleFilter()
{
    std::cout << "Particle filter destructor called" << std::endl;
}

Eigen::MatrixXd ParticleFilter::getParticles()
{
    return particles_;
}

Eigen::VectorXd ParticleFilter::diffdriveKinematicks(Eigen::VectorXd q, Eigen::VectorXd u, double dt)
{
    // !!! u = [wr, wl] !!!
    int n = q.size();
    int m = u.size();
    double r = 0.033;                           // turtlebot3 burger wheel radius
    double d = 0.16;                            // turtlebot3 burger distance between wheels
    double th = q(2);                           // theta orientation
    Eigen::VectorXd q_next(n);
    Eigen::MatrixXd A;                     // state matrix A
    A.resize(n,m);
    A << 0.5*r*cos(th), 0.5*r*cos(th), 0.5*r*sin(th), 0.5*r*sin(th), -0.5*r/d, 0.5*r/d;

    q_next = q + A*u*dt;

    // std::cout << "q_next: " << q_next.transpose() << std::endl;

    return q_next;
}


void ParticleFilter::predict(Eigen::VectorXd u, double dt)
{
    double sigma_x, sigma_y, sigma_th;
    sigma_x = stateCovariance_(0);
    sigma_y = stateCovariance_(1);
    sigma_th = stateCovariance_(2);

    std::default_random_engine gen;

    for (int i=0; i < n_; i++)
    {
        // Predict evolution of each particle
        Eigen::VectorXd q_next(3);
        q_next = diffdriveKinematicks(particles_.col(i), u, dt);

        // Add noise to each particle
        std::normal_distribution<double> dist_x(q_next(0), sigma_x);
        std::normal_distribution<double> dist_y(q_next(1), sigma_y);
        std::normal_distribution<double> dist_th(q_next(2), sigma_th);

        // Update particles
        particles_(0,i) = dist_x(gen);
        particles_(1,i) = dist_y(gen);
        particles_(2,i) = dist_th(gen);
    }
}
