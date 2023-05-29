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
        w_(i) = 1.0/n_;
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
    // leo rover
    double r = 0.065;
    double d = 0.313;                              
    // double r = 0.033;                           // turtlebot3 burger wheel radius
    // double d = 0.16;                            // turtlebot3 burger distance between wheels
    double th = q(2);                           // theta orientation
    Eigen::VectorXd q_next(n);
    Eigen::MatrixXd A;                     // state matrix A
    A.resize(n,m);
    A << 0.5*r*cos(th), 0.5*r*cos(th), 0.5*r*sin(th), 0.5*r*sin(th), -0.5*r/d, 0.5*r/d;

    q_next = q + A*u*dt;

    // std::cout << "q_next vs q: " << (q_next - q).transpose() << std::endl;

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

// Ackermann kinematics: q : [x,y,th] u : [v_lin]
Eigen::VectorXd ParticleFilter::ackermannKinematics(Eigen::VectorXd q, Eigen::VectorXd u, double dt)
{
    double v = u(0);
    double th = u(1);
    // std::cout << "Received velocity: " << v << ", " << th << std::endl;
    Eigen::Vector3d q_next;
    q_next(0) = q(0) + v*cos(th)*dt;
    q_next(1) = q(1) + v*sin(th)*dt;
    q_next(2) = th;
    // std::cout << "Predicted state: " << q_next.transpose() << std::endl;

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

void ParticleFilter::predictAckermann(Eigen::VectorXd u, double dt)
{
    double sigma_x, sigma_y, sigma_th;
    sigma_x = stateCovariance_(0);
    sigma_y = stateCovariance_(1);
    sigma_th = stateCovariance_(2);

    std::default_random_engine gen;

    for (int i=0; i < n_; i++)
    {
        // Predict evolution of each particle
        Eigen::Vector3d q_next;
        q_next = ackermannKinematics(particles_.col(i), u, dt);

        // std::cout << "Next state: " << q_next.transpose() << std::endl;
        // Add noise to each particle
        std::normal_distribution<double> dist_x(q_next(0), sigma_x);
        std::normal_distribution<double> dist_y(q_next(1), sigma_y);
        std::normal_distribution<double> dist_th(q_next(2), sigma_th);

        // Update particles
        particles_(0,i) = dist_x(gen);
        particles_(1,i) = dist_y(gen);
        particles_(2,i) = dist_th(gen);
    }

    // std::cout << "New set of particles: \n" << particles_.transpose() << std::endl;
}


void ParticleFilter::setWeights(Eigen::VectorXd weights)
{
    w_ = weights;
}

Eigen::VectorXd ParticleFilter::getWeights()
{
    return w_;
}


// update weight from the detection of a neighbor. Detection is assumed to be accurate
void ParticleFilter::updateWeights(Eigen::VectorXd observation, double sigma = 0.01)
{
    double total_weight = 0.0;
    std::cout << "Observed neighbor: " << observation.transpose() << std::endl;
    for (int i = 0; i < n_; i++)
    {
        Eigen::VectorXd p = particles_.col(i);
        // double likelihood = 1.0;
    
        Eigen::Vector2d obs = observation.head(2);

        // std::cout << "Landmark relative position: " << ld.transpose() << std::endl;

        double likelihood = std::exp(-0.5 * (pow((obs(0)-p(0)), 2) / pow(sigma, 2) + pow((obs(1)-p(1)), 2) / pow(sigma, 2)));
    

        w_(i) = likelihood;
        total_weight += w_(i);

        // std::cout << "w_" << std::to_string(i) << " = " << likelihood << std::endl;
    }

    std::cout << "Total weight: " << total_weight << std::endl;

    // normalize weights
    w_ = w_ / total_weight;
}


// observations is a std::vector of Eigen::Vector2d with detected landmarks in their position, 100.0 for undetected
// landmarks is a global std::vector of Eigen::Vector2d with known positions of landmarks
void ParticleFilter::updateWeights2(std::vector<Eigen::VectorXd> observations, double sigma = 0.1)
{
    std::vector<Eigen::Vector2d> landmarks(3);
    landmarks[0] = {3.5, 1.5};
    landmarks[1] = {5.0, 3.2};
    landmarks[2] = {3.0, 6.3};

    double total_weight = 0.0;
    for (int i = 0; i < n_; i++)
    {
        Eigen::VectorXd p = particles_.col(i);
        double likelihood = 1.0;
        for (int j = 0; j < observations.size(); j++)
        {
            if (observations[j](0) != 100.0 && observations[j](1) != 100.0)
            {
                Eigen::Vector2d obs = observations[j];

                // Get landmark's known position relative to actual particle
                Eigen::Vector2d ld;
                double th = p(2);
                double dx = landmarks[j](0) - p(0);
                double dy = landmarks[j](1) - p(1);

                ld(0) = dx * cos(th) + dy * sin(th);
                ld(1) = -dx * sin(th) + dy * cos(th);

                // std::cout << "Landmark relative position: " << ld.transpose() << std::endl;

                likelihood *= std::exp(-0.5 * (pow((obs(0)-ld(0)), 2) / pow(sigma, 2) + pow((obs(1)-ld(1)), 2) / pow(sigma, 2)));
            }
        }

        w_(i) = likelihood;
        total_weight += w_(i);

        // std::cout << "w_" << std::to_string(i) << " = " << likelihood << std::endl;
    }

    // normalize weights
    w_ = w_ / total_weight;
}

// observations is a std::vector of Eigen::Vector2d with detected landmarks in their position, 100.0 for undetected
// landmarks is a global std::vector of Eigen::Vector2d with known positions of landmarks
void ParticleFilter::updateWeights3(std::vector<Eigen::VectorXd> observations, std::vector<Eigen::VectorXd> global_lms, double sigma = 0.1)
{
    // std::vector<Eigen::Vector2d> landmarks(3);
    // landmarks[0] = {3.5, 1.5};
    // landmarks[1] = {5.0, 3.2};
    // landmarks[2] = {3.0, 6.3};

    double total_weight = 0.0;
    for (int i = 0; i < n_; i++)
    {
        Eigen::VectorXd p = particles_.col(i);
        double likelihood = 1.0;
        for (int j = 0; j < observations.size(); j++)
        {
            if (observations[j](0) != 100.0 && observations[j](1) != 100.0)
            {
                Eigen::Vector2d obs = observations[j];

                // Get landmark's known position relative to actual particle
                Eigen::Vector2d ld;
                double th = p(2);
                double dx = global_lms[j](0) - p(0);
                double dy = global_lms[j](1) - p(1);

                ld(0) = dx * cos(th) + dy * sin(th);
                ld(1) = -dx * sin(th) + dy * cos(th);

                // std::cout << "Landmark relative position: " << ld.transpose() << std::endl;

                likelihood *= std::exp(-0.5 * (pow((obs(0)-ld(0)), 2) / pow(sigma, 2) + pow((obs(1)-ld(1)), 2) / pow(sigma, 2)));
            }
        }

        w_(i) = likelihood;
        total_weight += w_(i);

        // std::cout << "w_" << std::to_string(i) << " = " << likelihood << std::endl;
    }

    // normalize weights
    w_ = w_ / total_weight;
}

void ParticleFilter::resample()
{
    std::default_random_engine gen;
    std::vector<double> weights;
    std::vector<Eigen::VectorXd> init_particles;
    std::vector<Eigen::VectorXd> resampled_particles(n_);

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
        int index = distribution(gen);
        resampled_particles[i] = init_particles[index];
        w_(i) = weights[index];
    }

    std::cout << "New particles defined" << std::endl;
    

    // Backwards conversion
    for (int i = 0; i < n_; i++)
    {
        particles_.col(i) = resampled_particles[i];
    }

    // std::cout << "New particles: \n" << particles_.transpose() << std::endl;

}


void ParticleFilter::resampleUniform()
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
        w_(i) = 1.0 / n_;
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
        w_(i) = 1.0 / n_;
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

Eigen::MatrixXd ParticleFilter::getCovariance()
{
    Eigen::MatrixXd cov_matrix(3,3);
    cov_matrix.setZero();
    Eigen::VectorXd mean = getMean();

    for (int i = 0; i < n_; i++)
    {
        Eigen::Vector3d p = particles_.col(i);
        Eigen::Vector3d diff = p - mean;
        cov_matrix += diff * diff.transpose();
    }

    cov_matrix = cov_matrix / (n_-1);

    return cov_matrix;
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


