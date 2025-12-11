#include "UDP_Interface.hpp"
#include "Robot_Sim.hpp"
using namespace std::chrono;

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "stmotion_controller_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        std::string config_fname, robot_name, root_pwd;
        double jpc_travel_time = 1.0;
        nh.getParam("config_fname", config_fname);
        ROS_INFO_STREAM("Config fname: " << config_fname);
        
        Eigen::MatrixXd cur_q, cur_qd, cur_qdd;
        
        ros::Rate loop_rate(150);
        unsigned int microsecond = 1000;

        stmotion_controller::math::VectorJd q;
        stmotion_controller::robot_sim::RobotSim::Ptr robot = std::make_shared<stmotion_controller::robot_sim::RobotSim>();
        robot->Setup(config_fname, nh);
        stmotion_controller::math::VectorJd jerk_ref = Eigen::MatrixXd::Zero(robot->robot_dof(), 1);
        jpc_travel_time = robot->get_travel_time();
        robot->set_JPC_speed(jpc_travel_time);
        robot->print_robot_property();

        ros::Subscriber jpc_travel_time_sub = nh.subscribe("/" + robot->get_name() + "/jpc_travel_time", 1, &stmotion_controller::robot_sim::RobotSim::jpcTravelTimeCallback, robot.get());
        ros::Publisher robot_state_pub = nh.advertise<std_msgs::Float32MultiArray>("/" + robot->get_name() + "/robot_state", robot->robot_dof() * 3); // pos, vel, acc
        ros::Subscriber goal_sub = nh.subscribe("/" + robot->get_name() + "/robot_goal", robot->robot_dof(), &stmotion_controller::robot_sim::RobotSim::goalCallback, robot.get());
        std_msgs::Float32MultiArray robot_state_msg;
        std_msgs::Float64 joint_msg;
       
        jerk_ref = robot->jpc(robot->get_goal());
        q = robot->step(jerk_ref, robot->get_goal());

        while(ros::ok)
        {
            // Update controller speed
            if(jpc_travel_time != robot->get_travel_time() && robot->get_travel_time() > 0)
            {  
                robot->set_JPC_speed(robot->get_travel_time());
                jpc_travel_time = robot->get_travel_time();
                  
                ROS_INFO_STREAM("Update jpc time");
                ROS_INFO_STREAM(jpc_travel_time);
            }
            
            // Get robot status
            robot_state_msg.data.clear();
            cur_q = robot->robot_q();
            cur_qd = robot->robot_qd();
            cur_qdd = robot->robot_qdd();
            jerk_ref = robot->jpc(robot->get_goal());
            
            for(int j=0; j<robot->robot_dof(); j++)
            {
                robot_state_msg.data.push_back(cur_q(j));
                robot_state_msg.data.push_back(cur_qd(j));
                robot_state_msg.data.push_back(cur_qdd(j));

                // Publish to simulation
                joint_msg.data = cur_q(j) / 180 * PI;
                robot->get_joint_pubs()[j].publish(joint_msg);
            }
            robot_state_pub.publish(robot_state_msg);
            q = robot->step(jerk_ref, robot->get_goal());
            usleep(7 * microsecond); // Pause <8ms to simulate the robot controller
            ros::spinOnce();
        }
        ROS_INFO_STREAM("Controller exit!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



