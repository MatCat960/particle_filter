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


void ParticleFilter::updateWeights(std::vector<Eigen::VectorXd> observations, double sigma = 0.1)
{
    // std::cout << "Starting set of particles: \n" << particles_.transpose() << std::endl;
    double weights_sum = 0.0;
    double den = 2 * M_PI * sigma * sigma;
    int keep_counter = 0;

    std::cout << "Observations: \n";
    for (int j = 0; j < observations.size(); j++)
    {
        std::cout << observations[j].transpose() << std::endl;
    }

    for (int i = 0; i < n_; i++)
    {
        bool keep = false;
        Eigen::VectorXd p = particles_.col(i);
        double wt = 1.0;

        for (int j = 0; j < observations.size(); j++)
        {
            Eigen::VectorXd obs = observations[j];
            // std::cout << "Actual observation: " << obs.transpose() << std::endl;
            double dist = sqrt(pow(obs(0)-p(0),2) + pow(obs(1)-p(1),2));      // distance particle -- observation
            std::cout << "Distance: " << dist << std::endl;
            if (dist < 3*sigma)
            {   
                keep = true;
            }
            // Update weight according to probability of a Multivariate Gaussian Distribution
            double num = exp(-(pow((obs(0)-p(0)), 2) / pow(sigma, 2) + pow((obs(1)-p(1)), 2) / pow(sigma, 2)));
            // std::cout << "Prob: " << num << std::endl;
            
            wt *= num;
        }



        weights_sum += wt;
        if (keep)
        {
            w_(i) = 1.0;
            keep_counter++;
        } else
        {
            w_(i) = wt;
        }
    }

    std::cout << "Particles removed: " << n_ - keep_counter << std::endl;
    // std::cout << "Weights sum: " << weights_sum << std::endl;

    // Normalize weights
    // for (int i = 0; i < n_; i++)
    // {
    //     w_(i) /= weights_sum;
    //     // std::cout << "Final weight of particle " << i << ": " << w_(i) << std::endl;
    // }
}


void ParticleFilter::resample()
{
    std::default_random_engine gen;
    std::vector<double> weights;
    std::vector<Eigen::VectorXd> init_particles;
    std::vector<Eigen::VectorXd> resampled_particles;

    // std::cout << "Weights: " << w_ << std::endl;

    for (int i = 0; i < n_; i++)
    {
        weights.push_back(w_(i));
        init_particles.push_back(particles_.col(i));
    }

    /* std::discrete_distribution produces random integers on the interval [0, n), where the probability of each individual integer i is defined as w
    i/S, that is the weight of the ith integer divided by the sum of all n weights. */
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    for (int i = 0; i < n_; i++)
    {
        resampled_particles.push_back(init_particles[distribution(gen)]);
    }

    std::cout << "New particles defined" << std::endl;

    // Backwards conversion
    for (int i = 0; i < n_; i++)
    {
        particles_.col(i) = resampled_particles[i];
    }

    // std::cout << "New particles: \n" << particles_.transpose() << std::endl;

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

void ParticleFilter::setState(Eigen::VectorXd q)
{
    state_ = q;
}

void ParticleFilter::remove_outliers(Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix, double threshold = 1e-7)
{
    // Identify outliers
    std::vector<Eigen::VectorXd> new_particles;

    double det = sqrt(pow(2*M_PI,2)*cov_matrix.determinant());
    for (int i = 0; i < particles_.cols(); i++)
    {
        Eigen::VectorXd p = particles_.col(i);
        Eigen::VectorXd diff = p - mean;
        double exponent = -0.5 * diff.transpose() * cov_matrix.inverse() * diff;
        double w = 1/det * exp(exponent);
        if (w > threshold)
        {
            new_particles.push_back(p);
        } else
        {
            std::cout << "Outlier identified in " << particles_.col(i).transpose() << std::endl;
        }
    }

    std::normal_distribution<double> dist_x(state_(0), cov_matrix(0,0));
    std::normal_distribution<double> dist_y(state_(1), cov_matrix(1,1));
    std::normal_distribution<double> dist_th(state_(2), cov_matrix(2,2));

    std::default_random_engine gen;

    // Generate new samples to replace removed outliers
    int needed_particles = n_ - new_particles.size();             // num of particles missing to reach the desired total number
    if (needed_particles > 0)
    {
        std::cout << "Generating " << needed_particles << " new particles" << std::endl;
        for (int j = 0; j < needed_particles; j++)
        {
            Eigen::VectorXd p(3);
            p(0) = dist_x(gen);
            p(1) = dist_y(gen);
            p(2) = dist_th(gen);
            new_particles.push_back(p);
        }
    }

    // Update particles
    setParticles(new_particles);
    
}


