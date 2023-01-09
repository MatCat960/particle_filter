#include <iostream>
#include <cmath>
#include <random>
#include <numeric>
#include <algorithm>
#include <vector>

#include "particle_filter/particle_filter.h"


ParticleFilter::ParticleFilter(int particles_num, Eigen::VectorXd initState, Eigen::VectorXd initCovariance)
{
    // std::cout << "Particle filter constructor called" << std::endl;
    std::cout << "Number of particles: " << particles_num << std::endl;
    r_sens_ = 4.0;
    n_ = particles_num;
    state_ = initState;
    size_t state_size = initState.size();
    w_.resize(n_);
    w_.setZero();
    particles_.resize(state_size, n_);
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
    // std::cout << "Particle fi"
}

ParticleFilter::~ParticleFilter()
{
    std::cout << "Particle filter destructor called" << std::endl;
}

Eigen::MatrixXd ParticleFilter::getParticles()
{
    return particles_;
}

void ParticleFilter::setParticles(Eigen::MatrixXd parts)
{
    // std::cout << "Entered particles setter." << std::endl;
    // particles_.clear();
    // std::cout << "Cleared\n";
    particles_.resize(parts.rows(), parts.cols());
    // std::cout << "Resized\n";
    for (int i=0; i<parts.cols(); i++)
    {
        particles_(0,i) = parts(0,i);
        particles_(1,i) = parts(1,i);
        particles_(2,i) = parts(2,i);
    }
}

void ParticleFilter::setParticles(std::vector<Eigen::VectorXd> parts)
{
    particles_.resize(parts[0].rows(), parts.size());
    // std::cout << "Resized\n";
    for (int i=0; i<parts.size(); i++)
    {
        // particles_(0,i) = parts(0,i);
        // particles_(1,i) = parts(1,i);
        // particles_(2,i) = parts(2,i);
        particles_.col(i) = parts[i];
    }
}

void ParticleFilter::setProcessCovariance(Eigen::VectorXd cov)
{
    stateCovariance_ = cov;
}

Eigen::VectorXd ParticleFilter::diffdriveKinematics(Eigen::VectorXd q, Eigen::VectorXd u, double dt)
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


Eigen::VectorXd ParticleFilter::UAVKinematics(Eigen::VectorXd q, Eigen::VectorXd u, double dt)
{
    int n = q.size();
    int m = u.size();
    Eigen::VectorXd q_next(n);
    Eigen::MatrixXd A;                     // state matrix A
    A.resize(n,m);
    A << 1, 0, 0, 0, 1, 0, 0, 0, 0;

    q_next = q + A*u*dt;

    // std::cout << "q_next: " << q_next.transpose() << std::endl;

    return q_next;
}


Eigen::MatrixXd ParticleFilter::multiDiffdriveKinematics(Eigen::MatrixXd q, Eigen::MatrixXd u, double dt)
{
    // !!! u = [wl, wr] !!!
    // Each column of q and u is a state and input vector
    Eigen::MatrixXd q_next(q.rows(), q.cols());
    for (int i=0; i<q.cols(); i++)
    {
        q_next.col(i) = diffdriveKinematics(q.col(i), u.col(i), dt);
    }

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
        q_next = diffdriveKinematics(particles_.col(i), u, dt);

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

void ParticleFilter::predictUAV(Eigen::VectorXd u, double dt)
{
    if (u.size() < 3)
    {
        for (int i = u.size(); i < 3; i++)
        {
            u.conservativeResize(u.size()+1);
            u(i) = 0.0;
        }
    }
    double sigma_x, sigma_y, sigma_th;
    sigma_x = stateCovariance_(0);
    sigma_y = stateCovariance_(1);
    sigma_th = stateCovariance_(2);

    std::default_random_engine gen;

    for (int i=0; i < n_; i++)
    {
        // Predict evolution of each particle
        Eigen::VectorXd q_next(3);
        q_next = UAVKinematics(particles_.col(i), u, dt);

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



std::vector<Eigen::VectorXd> ParticleFilter::resample(Eigen::VectorXd q, double fov, double r_sens)
{
    std::vector<Eigen::VectorXd> particles_resampled;
    size_t size = q.size();

    // Delete particles inside fov (I would see it)
    // for (int i=0; i<n_; i++)
    // {
    //     if (!insideFOV(q, particles_.col(i), fov, r_sens))
    //     {
    //         Eigen::VectorXd p(size);
    //         p(0) = particles_(0,i);
    //         p(1) = particles_(1,i);
    //         p(2) = particles_(2,i);
    //         particles_resampled.push_back(p);
    //     }
    // }

    int needed_particles = n_ - particles_resampled.size();             // num of particles missing to reach the desired total number
    

    // ---- Get distribution of particles outside fov (GMM) -----


    // Generate new particles according to the GMM distribution
    // Add new particles to the resampled particles to get 1000 total

}


void ParticleFilter::matchObservation(Eigen::VectorXd q)
{
    // Generate new particles based on observation
    // Default covariance is very small assuming the observation is perfect, set it higher if you want to add noise to the observation
    state_ = q;
    stateCovariance_ = 0.1*Eigen::VectorXd::Ones(3);

    double sigma_x, sigma_y, sigma_th;
    sigma_x = stateCovariance_(0);
    sigma_y = stateCovariance_(1);
    sigma_th = stateCovariance_(2);

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
}

Eigen::VectorXd ParticleFilter::getMean()
{
    Eigen::VectorXd mean(3);
    mean(0) = particles_.row(0).mean();
    mean(1) = particles_.row(1).mean();
    mean(2) = particles_.row(2).mean();

    return mean;
}


Eigen::VectorXd ParticleFilter::getState()
{
    return state_;
}
