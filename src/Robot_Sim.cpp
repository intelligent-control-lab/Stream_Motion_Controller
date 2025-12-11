#include "Robot_Sim.hpp"

namespace stmotion_controller
{
namespace robot_sim
{
RobotSim::RobotSim()
{
}
void RobotSim::print_robot_property()
{
    ROS_INFO_STREAM(robot_name_);
    // std::cout << "Robot Pos:" << q_ << std::endl;
    for(int i=0; i<njoints_; i++)
    {
        ROS_INFO_STREAM(joint_topics_[i]);
    }
    ROS_INFO_STREAM("");
}
        
void RobotSim::Setup(const std::string& config_fname, ros::NodeHandle& nh)
{
    std::ifstream config_file(config_fname, std::ifstream::binary);
    Json::Value config;
    config_file >> config;
    njoints_ = config["Num_Joints"].asInt();
    robot_name_ = config["Robot_Name"].asString();
    jpc_travel_time_ = config["JPC_Travel_Time"].asDouble();
    for(int i=1; i<=njoints_; i++)
    {
        std::string jnt_topic = "/" + robot_name_ + "/" + config["Joint_Names"][std::to_string(i)].asString() + "_position_controller/command";
        joint_topics_.push_back(jnt_topic);

        ros::Publisher j_pub = nh.advertise<std_msgs::Float64>(jnt_topic, 1);
        joint_pubs_.push_back(j_pub);
    }
    controller_goal_ = Eigen::MatrixXd::Zero(njoints_, 1);
    

    thetamax_.resize(njoints_, 2);
    thetadotmax_.resize(njoints_, 1);
    q_.resize(njoints_, 1);
    qd_.resize(njoints_, 1);
    qdd_.resize(njoints_, 1);
    for(int i=0; i<njoints_; i++)
    {
        thetadotmax_.row(i) << 35 * PI / 180;
        thetamax_.row(i) << 170 * PI / 180, -170 * PI / 180;
        q_.row(i) << 0.0;
        qd_.row(i) << 0.0;
        qdd_.row(i) << 0.0;
    }
    
  
    Adt_.resize(3, 3);
    Bdt_.resize(3, 1);
    Adt_ << 1, delta_t_, 0.5 * pow(delta_t_, 2), 
            0, 1, delta_t_, 
            0, 0, 1;
    Bdt_ << pow(delta_t_, 3) / 6.0, 0.5 * pow(delta_t_, 2), delta_t_;

    jpc_step_ = Eigen::MatrixXd::Constant(njoints_, 1, -1);
    jpc_ubuffer_ = Eigen::MatrixXd::Zero(njoints_, jpc_buffer_len_);
    A_ = Eigen::MatrixXd::Identity(3, 3);
    B_ = Eigen::MatrixXd::Zero(3, jpc_buffer_len_);

    for(int i=0; i<jpc_buffer_len_; i++)
    {
        B_.col(jpc_buffer_len_ - i - 1) << A_ * Bdt_;
        A_ = Adt_ * A_;
    }
    Binv_ = math::PInv(B_);

    std::cout << std::setprecision(5);
    std::cout << "Robot Setup Done!" << std::endl;
}

void RobotSim::set_JPC_speed(const double& t)
{
    jpc_buffer_len_ = round(t * 125);
    jpc_ubuffer_ = Eigen::MatrixXd::Zero(njoints_, jpc_buffer_len_);
    A_ = Eigen::MatrixXd::Identity(3, 3);
    B_ = Eigen::MatrixXd::Zero(3, jpc_buffer_len_);

    for(int i=0; i<jpc_buffer_len_; i++)
    {
        B_.col(jpc_buffer_len_ - i - 1) << A_ * Bdt_;
        A_ = Adt_ * A_;
    }
    Binv_ = math::PInv(B_);
    
    for(int i=0; i<njoints_; i++)
    {
        jpc_step_(i) = -1;
    }
}

bool RobotSim::is_static()
{
    for(int i=0; i<njoints_; i++)
    {
        if(abs(qd_(i)) > epsilon_ || abs(qdd_(i)) > epsilon_)
        {
            return false;
        }
    }
    return true;
}


bool RobotSim::reached_goal(math::VectorJd goal)
{
    for(int i=0; i<njoints_; i++)
    {
        if(abs(q_(i) - goal(i)) > epsilon_)
        {
            return false;
        }
    }
    return true;
}

math::VectorJd RobotSim::jpc(const math::VectorJd& goal)
{
    math::VectorJd jerk = Eigen::MatrixXd::Zero(njoints_, 1);
    Eigen::MatrixXd X(3, 1);
    Eigen::MatrixXd G(3, 1);
    Eigen::MatrixXd tmp(3, 1);
    for(int idx=0; idx<njoints_; idx++)
    {
        if((goal(idx) != q_(idx) && jpc_step_(idx) < 0) || jpc_step_(idx) >= 0)
        {   
            if(jpc_step_(idx) < 0) // Plan
            {
                X << q_(idx), qd_(idx), qdd_(idx);
                G << goal(idx), 0, 0;
                tmp = G - A_ * X;
                jpc_ubuffer_.row(idx) = (Binv_ * tmp).transpose();
                jpc_step_(idx) = 0;
                jerk(idx) = jpc_ubuffer_.coeff(idx, jpc_step_(idx));
            }
            else if(jpc_step_(idx) < jpc_buffer_len_ - 1) // Read next jerk
            {
                jpc_step_(idx) ++;
                jerk(idx) = jpc_ubuffer_.coeff(idx, jpc_step_(idx));
            }
            else // zero
            {
                jpc_step_(idx) ++;
                jerk(idx) = 0;
            }
        }
    }
    return jerk;
}


math::VectorJd RobotSim::step(const math::VectorJd& jerk, const math::VectorJd& goal)
{
    Eigen::MatrixXd X(3, 1);
    math::VectorJd pos_out = jerk;
    Eigen::MatrixXd unew(3, 1);
    for(int i=0; i<njoints_; i++)
    {
        if(jpc_step_(i) == jpc_buffer_len_ - 1)
        {
            pos_out(i) = goal(i);
            q_(i) = goal(i);
            qd_(i) = 0;
            qdd_(i) = 0;
            jpc_step_(i) = -1;
        }
        else
        {
            X << q_(i), qd_(i), qdd_(i);
            unew = Adt_ * X + Bdt_ * jerk(i);
            pos_out(i) = unew(0);
            q_(i) = pos_out(i);
            qd_(i) = unew(1);
            qdd_(i) = unew(2);
        }
    }
    return pos_out;
}

void RobotSim::goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for (int i = 0; i < njoints_; i++) 
    {
        controller_goal_(i) = msg->data[i];
    }
}

void RobotSim::jpcTravelTimeCallback(const std_msgs::Float64::ConstPtr& msg)
{
    jpc_travel_time_ = msg->data;
}


}
}
