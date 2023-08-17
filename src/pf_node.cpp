#include <time.h>
#include <chrono>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <signal.h>
#include <fstream>
#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "particle_filter/particle_filter.h"
// #include "Graphics.h"

#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

//------------------------------------------------------------------------
sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed
//------------------------------------------------------------------------


const double VMAX_LIN = 1.0;
const double VMAX_ANG = 1.0;

class Controller
{
    public:
        // initialize filter with starting position and covariance!!
        Controller(): nh_("~"), filter(PARTICLES_NUM, Eigen::Vector3d::Zero(), 0.5*Eigen::Vector3d::Ones())
        {
            // get params from launch file
            this->nh_.getParam("GRAPHICS_ON", GRAPHICS_ON);
            this->nh_.getParam("USE_GROUND_TRUTH", USE_GROUND_TRUTH);
            this->nh_.getParam("FILE_PATH", FILE_PATH);
            this->nh_.getParam("VERBOSE", VERBOSE);
            this->nh_.getParam("frame", frame);


            pf_pub_ = nh_.advertise<nav_msgs::Odometry>("pf_estimate",10);
            vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, std::bind(&Controller::velCallback, this, std::placeholders::_1));
            detections_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/detections", 1, std::bind(&Controller::detectionsCallback, this, std::placeholders::_1));
            
            if (USE_GROUND_TRUTH)
            {
                realpose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth", 1, std::bind(&Controller::realposeCallback, this, std::placeholders::_1));
            }

            // publish particles for RViz visualization
            if (GRAPHICS_ON)
            {
                particles_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("particles",1);
            }
            timer_ = nh_.createTimer(ros::Duration(0.25), std::bind(&Controller::timerCallback, this));

            // read global landmark positions from file
            ld_file.open(FILE_PATH);
            if (ld_file.is_open())
            {
                std::string line;
                while(std::getline(ld_file, line))
                {
                    std::istringstream iss(line);
                    std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
                    Eigen::VectorXd lm(2);
                    lm(0) = std::stod(results[0]);
                    lm(1) = std::stod(results[1]);
                    lm_global.push_back(lm);
                }
            }

            std::cout << "Global known position of landmarks: \n";
            for (int i = 0; i < lm_global.size(); i++)
            {
                std::cout << lm_global[i].transpose() << std::endl;
            }

            // initialize detections vector with (100.0, 100.0) --> no landmark detected
            for (int i = 0; i < lm_global.size(); i++)
            {
                detections.push_back(100*Eigen::VectorXd::Ones(2));
            }

            // initialize variables to zero
            vel = Eigen::Vector2d::Zero();
            realpose = Eigen::Vector3d::Zero();

            initialized = false;

            std::cout << "Initialization completed. \n";
        }

        ~Controller()
        {
            std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
        }

        void timerCallback();
        void velCallback(const geometry_msgs::Twist::ConstPtr msg);
        void detectionsCallback(const geometry_msgs::PoseArray::ConstPtr msg);
        void realposeCallback(const nav_msgs::Odometry::ConstPtr msg);

    
    private:
        int PARTICLES_NUM = 5000;
        bool GRAPHICS_ON = false;
        bool USE_GROUND_TRUTH = false;
        bool VERBOSE = false;
        std::string frame = "map";
        std::string FILE_PATH;
        ParticleFilter filter;
        bool initialized;
        ros::Publisher pf_pub_;
        ros::Publisher particles_pub_;
        ros::Subscriber vel_sub_;
        ros::Subscriber detections_sub_;
        ros::Subscriber realpose_sub_;
        ros::Timer timer_;
        ros::NodeHandle nh_;

        std::vector<Eigen::VectorXd> lm_global;          // global landmark position
        std::vector<Eigen::VectorXd> detections;
        Eigen::Vector2d vel;                // vel = [v_lin, orientation]
        Eigen::Vector3d realpose;           // only for visualization   

        std::ifstream ld_file;
};

void Controller::velCallback(const geometry_msgs::Twist::ConstPtr msg)
{
    // std::cout << "Velocity callback" << std::endl;
    this->vel(0) = msg->linear.x;                                // linear vel is a value in [0, 100]
    this->vel(1) = msg->angular.z;                              // steering angle is a value in [-180, 180] degrees --> conversion to rad 
    // std::cout << "Vel: " << this->vel.transpose() << std::endl;
}

void Controller::realposeCallback(const nav_msgs::Odometry::ConstPtr msg)
{
    // std::cout << "Real pose callback \n";
    this->realpose(0) = msg->pose.pose.position.x;
    this->realpose(1) = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->realpose(2) = yaw;   
}

void Controller::detectionsCallback(const geometry_msgs::PoseArray::ConstPtr msg)
{
    for (int i = 0; i < msg->poses.size(); i++)
    {
        // std::cout << "Detection callback. \n";
        Eigen::Vector2d det;
        det(0) = msg->poses[i].position.x;
        det(1) = msg->poses[i].position.y;
        this->detections[i] = det;
        // std::cout << "Detected landmarks: " << det.transpose() << std::endl;
    }
}


void Controller::timerCallback()
{
    /* ---------------------------------
    bool ld_found = false;
    for (auto det : this->detections)
    {
        if (det(0) != 100.0 && det(1) != 100.0)
        {
            ld_found = true;
        }
    }
    if (!ld_found)
    {
        "No landmark found. Skipping particle filtering ...\n";
        return;
    }
    ------------------------ */
    // std::cout << "Main loop." << std::endl;
    if (VERBOSE)
    {
        std::cout << "Detected landmarks: \n";
        for (int i = 0; i < this->detections.size(); i++)
        {
            auto det = this->detections[i];
            if (det(0) != 100.0 && det(1) != 100.0)
            {
                std::cout << "Landmark " << i << ": " << det.transpose() << std::endl;
            }
            // std::cout << "Actual velocity: " << this->vel.transpose() << std::endl;
        }
    }
    
    double dt = 0.25;
    double sigma = 1.0;
    filter.predict(this->vel, dt);              // change to correct kinematic model
    // std::cout << "Prediction completed" << std::endl;
    std::cout << "Predicted position : " << filter.getMean().transpose() << std::endl;
    filter.updateWeights3(this->detections, lm_global, 0.1);
    // std::cout << "Weights updated" << std::endl;
    filter.resample();

    // Publish pose estimation
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = frame;
    pose_msg.pose.pose.position.x = filter.getMean()(0);
    pose_msg.pose.pose.position.y = filter.getMean()(1);
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, filter.getMean()(2));
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    this->pf_pub_.publish(pose_msg);



    if (VERBOSE)
    {
        // std::cout << "Particles: \n" << filter.getParticles().transpose() << std::endl;
        std::cout << "Estimated position: " << filter.getMean().transpose() << std::endl;
    }
    
    if (USE_GROUND_TRUTH)
    {
        double error = sqrt(pow(this->realpose(0)-filter.getMean()(0),2) + pow(this->realpose(1)-filter.getMean()(1),2));
        double error2 = sqrt(pow(this->realpose(0)-filter.getMean()(0),2) + pow(this->realpose(1)-filter.getMean()(1),2));
        if (VERBOSE)
        {
            std::cout << "Estimation error: " << error2 << std::endl;
        }
    }

    if (GRAPHICS_ON)
    {
        // std::cout << "Publishing particles" << std::endl;
        visualization_msgs::MarkerArray particles;
        particles.markers.resize(PARTICLES_NUM);
        for (int i = 0; i < PARTICLES_NUM; i++)
        {
            particles.markers[i].header.frame_id = frame;
            particles.markers[i].header.stamp = ros::Time::now();
            particles.markers[i].ns = "particles";
            particles.markers[i].id = i;
            particles.markers[i].type = visualization_msgs::Marker::SPHERE;
            particles.markers[i].action = visualization_msgs::Marker::ADD;
            particles.markers[i].pose.position.x = filter.getParticles()(0,i);
            particles.markers[i].pose.position.y = filter.getParticles()(1,i);
            particles.markers[i].pose.position.z = 0.0;
            particles.markers[i].pose.orientation.x = 0.0;
            particles.markers[i].pose.orientation.y = 0.0;
            particles.markers[i].pose.orientation.z = 0.0;
            particles.markers[i].pose.orientation.w = 1.0;
            particles.markers[i].scale.x = 0.1;
            particles.markers[i].scale.y = 0.1;
            particles.markers[i].scale.z = 0.1;
            particles.markers[i].color.a = 1.0;
            particles.markers[i].color.r = 0.0;
            particles.markers[i].color.g = 0.0;
            particles.markers[i].color.b = 1.0;
        }
        particles_pub_.publish(particles);
    }
}

std::shared_ptr<Controller> globalobj_signal_handler;     //the signal function requires only one argument {int}, so the class and its methods has to be global to be used inside the signal function.
void nodeobj_wrapper_function(int){
    std::cout<<"signal handler function CALLED"<<std::endl;
    node_shutdown_request = 1;
}


int main(int argc, char * argv[])
{
    signal(SIGINT, nodeobj_wrapper_function);
    ros::init(argc, argv, "pf_localization", ros::init_options::NoSigintHandler);
    auto node = std::make_shared<Controller>();

    while(!node_shutdown_request){
        ros::spinOnce();
    }

    if(ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.\n");
        ros::shutdown();
    }

    return 0;
}
