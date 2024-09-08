#include "UDP_Interface.hpp"
#include "Robot.hpp"
using namespace std::chrono;

Eigen::MatrixXd cart_T_teleop = -1 * Eigen::MatrixXd::Identity(4, 4);

void teleopCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    cart_T_teleop << msg->data[0], msg->data[1], msg->data[2], msg->data[3],
            msg->data[4], msg->data[5], msg->data[6], msg->data[7],
            msg->data[8], msg->data[9], msg->data[10], msg->data[11],
            msg->data[12], msg->data[13], msg->data[14], msg->data[15];
}

int main(int argc, char **argv)
{
    try
    {
        bool use_robot, ssa_enb;
        ros::init(argc, argv, "stmotion_controller_node_teleoperation");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        std::string config_fname, root_pwd, DH_fname, DH_tool_fname, robot_base_fname, robot_ip, nominal_mode
                    , controller_joint_goal_topic, controller_time_topic;
        nh.getParam("config_fname", config_fname);
        nh.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        DH_fname = root_pwd + config["DH_fname"].asString();
        robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        DH_tool_fname = root_pwd + config["DH_tool_fname"].asString();
        controller_joint_goal_topic = config["ST_Controller_Topic"]["Joint_Goal"].asString();
        controller_time_topic = config["ST_Controller_Topic"]["Controller_Time"].asString();
        nominal_mode = config["Nominal_mode"].asString();
        Eigen::MatrixXd cur_q, cur_qd, cur_qdd;
        ROS_INFO_STREAM("Config fname: " << config_fname);
        ROS_INFO_STREAM("Root pwd: " << root_pwd);
        ROS_INFO_STREAM("DH fname: " << DH_fname);
        ROS_INFO_STREAM("Robot base fname: " << robot_base_fname);
        
        ros::Rate loop_rate(150);
        unsigned int microsecond = 1000;

        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup(DH_fname, robot_base_fname);
        robot->set_DH_tool(DH_tool_fname);
        robot->print_robot_property();
        ros::Subscriber teleop_sub = nh.subscribe("/stmotion_controller_bringup/robot_teleop", 16, teleopCallback);
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>(controller_joint_goal_topic, robot->robot_dof());
        ros::Publisher controller_time_pub = nh.advertise<std_msgs::Float64>(controller_time_topic, 1);

        bool IK_status;
        std_msgs::Float32MultiArray goal_msg;
        std_msgs::Float64 controller_time_msg;
        Eigen::MatrixXd home_q(robot->robot_dof(), 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0;
        stmotion_controller::math::VectorJd cur_goal = home_q;
        stmotion_controller::math::VectorJd cur_goal_teleop = cur_goal;
        
        while(ros::ok)
        {
            Eigen::MatrixXd cart_T_current = stmotion_controller::math::FK(cur_goal, robot->robot_DH(), robot->robot_base(), false);
            
            if(cart_T_teleop(3,3) == 1.0)
            { 
                cart_T_current.block(0, 0, 3, 3) = cart_T_teleop.block(0, 0, 3, 3);
                cart_T_current.block(0, 3, 3, 1) = cart_T_current.block(0, 3, 3, 1) + cart_T_teleop.block(0, 3, 3, 1);

                // cart_T_current(0, 3) = std::min(std::max(cart_T_current(0, 3), 0.4), 0.8);
                // cart_T_current(1, 3) = std::min(std::max(cart_T_current(1, 3), -0.3), 0.3);
                // cart_T_current(2, 3) = std::min(std::max(cart_T_current(2, 3), 0.3), 0.7);
            }
            
            stmotion_controller::math::VectorJd cur_goal_teleop =  stmotion_controller::math::IK_closed_form(cur_goal, cart_T_current, robot->robot_DH(), 
                                                                        robot->robot_base_inv(), robot->robot_ee_inv(), 0, IK_status);
            
            // IK(const VectorJd& q, const Eigen::Matrix<double, 3, 1>& cart_goal, const Eigen::Matrix3d rot_goal, 
            // const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad, const uint& max_iter, const double& step)
            // ROS_INFO_STREAM(cart_T_current);
            // cur_goal_teleop =  stmotion_controller::math::IK(cur_goal, cart_T_current.block(0, 3, 3, 1), cart_T_current.block(0, 0, 3, 3), 
            //                                                                                      robot->robot_DH(), 
            //                                                                                      robot->robot_base(), 0, 10000, 0.001);

            bool invalid = false;
            for(int j=0; j<robot->robot_dof(); j++) {
                if(std::isnan(cur_goal_teleop(j))) 
                {
                    invalid = true;
                    break;
                }
                if(std::abs(cur_goal_teleop(1) - cur_goal(1)) > 30) 
                {
                    invalid = true;
                    break;
                }
            }

            if(!invalid)
            {
                cur_goal = cur_goal_teleop;
            }
            ROS_INFO_STREAM("###########");
            ROS_INFO_STREAM(cur_goal);
            ROS_INFO_STREAM(cur_goal_teleop);
            
            goal_msg.data.clear();
            for(int j=0; j<robot->robot_dof(); j++)
            {
                goal_msg.data.push_back(cur_goal(j));
            }
            goal_pub.publish(goal_msg);
            // sleep(1);
            ros::spinOnce();
        }
        ROS_INFO_STREAM("Teleoperation Controller exit!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



