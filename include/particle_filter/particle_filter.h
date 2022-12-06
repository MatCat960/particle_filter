#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>


class ParticleFilter
{
    private:
        int n_;                                 // number of particles
        bool initialized_;                      // flag for initialization
        Eigen::VectorXd state_;               // state vector
        Eigen::VectorXd w_;                     // weights of particles
        Eigen::MatrixXd particles_;             // particles matrix -- 1 particle (x,y,th) per column
        Eigen::VectorXd stateCovariance_;                 // noise vector (x,y,th)

    public:
        ParticleFilter(int particles_num, Eigen::VectorXd initState, Eigen::VectorXd initCovariance);       // constructor
        ~ParticleFilter();                                              // destructor
        void predict(Eigen::VectorXd u, double dt);   // prediction step
        Eigen::VectorXd diffdriveKinematicks(Eigen::VectorXd q, Eigen::VectorXd u, double dt);
        void updateWeights();
        void resample();
        Eigen::MatrixXd getParticles();

};

#endif // KALMAN_FILTER_H