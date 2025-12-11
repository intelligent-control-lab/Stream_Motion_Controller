#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"

namespace stmotion_controller
{
namespace robot_sim
{
class RobotSim
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<RobotSim> Ptr;
        typedef std::shared_ptr<RobotSim const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        int njoints_ = 21;
        double delta_t_ = 0.008; // s. Traj frequency
        std::string robot_name_ = "Galaxea R1-Lite";
        std::vector<std::string> joint_topics_;
        std::vector<ros::Publisher> joint_pubs_;
        
        Eigen::MatrixXd q_;
        Eigen::MatrixXd qd_;
        Eigen::MatrixXd qdd_;
        math::VectorJd controller_goal_ = Eigen::MatrixXd::Zero(njoints_, 1);
        double jpc_travel_time_ = 10.0;

        Eigen::MatrixXd thetamax_; // njoints_ x 2
        Eigen::MatrixXd thetadotmax_; // njoints_ x 2
        double epsilon_ = 0.00001;
        math::VectorJd jerk_max_ = Eigen::MatrixXd::Constant(njoints_, 1, 4000.0);
        
        int jpc_buffer_len_ = 60;
        Eigen::MatrixXd jpc_step_;
        Eigen::MatrixXd jpc_ubuffer_;
        Eigen::MatrixXd A_;
        Eigen::MatrixXd B_;
        Eigen::MatrixXd Binv_;
        Eigen::MatrixXd Adt_;
        Eigen::MatrixXd Bdt_;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:
        

    public:
        RobotSim();
        ~RobotSim(){}

        // setter
        void set_robot_q(const math::VectorJd& q) {q_ = q;};
        void set_robot_qd(const math::VectorJd& qd) {qd_ = qd;};
        void set_robot_qdd(const math::VectorJd& qdd) {qdd_ = qdd;};
        void set_JPC_speed(const double& t);
        void set_robot_name(const std::string& name) {robot_name_ = name;};
        
        // getter
        void print_robot_property();
        int robot_dof() {return njoints_;};
        Eigen::MatrixXd robot_q() {return q_;};
        Eigen::MatrixXd robot_qd() {return qd_;};
        Eigen::MatrixXd robot_qdd() {return qdd_;};
        math::VectorJd get_goal() {return controller_goal_;};
        double get_travel_time() {return jpc_travel_time_;};
        std::string get_name() {return robot_name_;};
        std::vector<ros::Publisher> get_joint_pubs() {return joint_pubs_;};


        // Operations
        void Setup(const std::string& config_fname, ros::NodeHandle& nh);
        math::VectorJd jpc(const math::VectorJd& goal);
        math::VectorJd step(const math::VectorJd& jerk, const math::VectorJd& goal);
        bool is_static();
        bool reached_goal(math::VectorJd goal);

        // Callbacks
        void goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void jpcTravelTimeCallback(const std_msgs::Float64::ConstPtr& msg);

};
}
}