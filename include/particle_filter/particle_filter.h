#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

// #include "GaussianMixtureModel/GaussianMixtureModel.h"


class ParticleFilter
{
    private:
        double r_sens_;         // sensing range
        int n_;                                 // number of particles
        bool initialized_;                      // flag for initialization
        Eigen::VectorXd state_;               // state vector
        Eigen::VectorXd w_;                     // weights of particles
        Eigen::MatrixXd particles_;             // particles matrix -- 1 particle (x,y,th) per column
        Eigen::VectorXd stateCovariance_;                 // noise vector (x,y,th)
        // gauss::gmm::GaussianMixtureModel gmm_;   // Gaussian Mixture Model 

    public:
        ParticleFilter(int particles_num, Eigen::VectorXd initState, Eigen::VectorXd initCovariance);       // constructor
        ~ParticleFilter();                                              // destructor
        void predict(Eigen::VectorXd u, double dt);   // prediction step
        void predictUAV(Eigen::VectorXd u, double dt);
        void predictAckermann(Eigen::VectorXd u, double dt);
        Eigen::VectorXd diffdriveKinematics(Eigen::VectorXd q, Eigen::VectorXd u, double dt);
        Eigen::VectorXd ackermannKinematics(Eigen::VectorXd q, Eigen::VectorXd u, double dt);
        Eigen::MatrixXd multiDiffdriveKinematics(Eigen::MatrixXd q, Eigen::MatrixXd u, double dt);
        Eigen::VectorXd UAVKinematics(Eigen::VectorXd q, Eigen::VectorXd u, double dt);
        void updateWeights(Eigen::VectorXd observation, double sigma);
        void updateWeights2(std::vector<Eigen::VectorXd> observations, double sigma);
        void updateWeights3(std::vector<Eigen::VectorXd> observations, std::vector<Eigen::VectorXd> global_lms, double sigma);
        void resample();         // outputs new set of particles
        void resampleUniform();
        Eigen::MatrixXd getParticles();
        void setWeights(Eigen::VectorXd weights);
        Eigen::VectorXd getWeights();
        void setParticles(Eigen::MatrixXd parts);
        void setParticles(std::vector<Eigen::VectorXd> parts);
        void setProcessCovariance(Eigen::VectorXd cov);             // set process covariance as vector (= diag matrix)
        void matchObservation(Eigen::VectorXd q);                   // update particles based on observation
        Eigen::VectorXd getState();
        void setState(Eigen::VectorXd q);
        Eigen::VectorXd getMean();
        Eigen::MatrixXd getCovariance();
        void remove_outliers(Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix, double threshold);
        void updateParticlesNumber(int n);
        int getParticlesNumber();
        

};

#endif // PARTICLE_FILTER_H