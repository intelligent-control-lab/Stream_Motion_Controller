#include "UDP_Interface.hpp"
#include "Robot.hpp"
using namespace std::chrono;

std::vector<stmotion_controller::math::Capsule> human_cap(6);
stmotion_controller::math::VectorJd controller_goal = Eigen::MatrixXd::Zero(6, 1);
double new_jpc_travel_time = 10.0;

void goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    controller_goal << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
}

// Get human state from sensor (camera/avp)
void humanStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i<6; i++) 
    {
        human_cap[i].r = msg->data[i*7];
        human_cap[i].p.col(0) << msg->data[i*7 + 1], msg->data[i*7 + 2], msg->data[i*7 + 3];
        human_cap[i].p.col(1) << msg->data[i*7 + 4], msg->data[i*7 + 5], msg->data[i*7 + 6];
    }
}

void jpcTravelTimeCallback(const std_msgs::Float64::ConstPtr& msg)
{
    new_jpc_travel_time = msg->data;
}

int main(int argc, char **argv)
{
    try
    {
        bool use_robot, ssa_enb;
        double jpc_travel_time = 0.0;
        ros::init(argc, argv, "stmotion_controller_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        std::string config_fname, root_pwd, DH_fname, robot_base_fname, robot_ip, nominal_mode;
        std::string j1_topic, j2_topic, j3_topic, j4_topic, j5_topic, j6_topic;
        nh.getParam("config_fname", config_fname);
        nh.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        DH_fname = root_pwd + config["DH_fname"].asString();
        robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        ssa_enb = config["SSA_Enable"].asBool();
        use_robot = config["Use_Robot"].asBool();
        jpc_travel_time = config["JPC_travel_time"].asDouble();
        robot_ip = config["Robot_IP"].asString();
        j1_topic = config["Simulation_controller_topic"]["J1"].asString();
        j2_topic = config["Simulation_controller_topic"]["J2"].asString();
        j3_topic = config["Simulation_controller_topic"]["J3"].asString();
        j4_topic = config["Simulation_controller_topic"]["J4"].asString();
        j5_topic = config["Simulation_controller_topic"]["J5"].asString();
        j6_topic = config["Simulation_controller_topic"]["J6"].asString();
        nominal_mode = config["Nominal_mode"].asString();
        new_jpc_travel_time = jpc_travel_time;
        Eigen::MatrixXd cur_q, cur_qd, cur_qdd;
        ROS_INFO_STREAM("Config fname: " << config_fname);
        ROS_INFO_STREAM("Root pwd: " << root_pwd);
        ROS_INFO_STREAM("DH fname: " << DH_fname);
        ROS_INFO_STREAM("Robot base fname: " << robot_base_fname);
        ROS_INFO_STREAM("SSA_Enable: " << ssa_enb);
        ROS_INFO_STREAM("Robot IP: " << robot_ip);
        ROS_INFO_STREAM("J1 topic: " << j1_topic);
        ROS_INFO_STREAM("J2 topic: " << j2_topic);
        ROS_INFO_STREAM("J3 topic: " << j3_topic);
        ROS_INFO_STREAM("J4 topic: " << j4_topic);
        ROS_INFO_STREAM("J5 topic: " << j5_topic);
        ROS_INFO_STREAM("J6 topic: " << j6_topic);
        ROS_INFO_STREAM("Nominal controller mode: " << nominal_mode);
        ros::Rate loop_rate(150);
        unsigned int microsecond = 1000;

        stmotion_controller::udp::UDP_Interface::Ptr robot_connection = std::make_shared<stmotion_controller::udp::UDP_Interface>();
        stmotion_controller::math::VectorJd q;
        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup(DH_fname, robot_base_fname);
        robot->set_JPC_speed(jpc_travel_time);
        robot->print_robot_property();
        stmotion_controller::math::VectorJd jerk_ref = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::math::VectorJd jerk_safe = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::udp::recv_pack recv_packet;
        ros::Subscriber jpc_travel_time_sub = nh.subscribe("jpc_travel_time", 1, jpcTravelTimeCallback);
        ros::Publisher robot_state_pub = nh.advertise<std_msgs::Float32MultiArray>("robot_state", robot->robot_dof() * 3); // pos, vel, acc
        ros::Subscriber goal_sub = nh.subscribe("robot_goal", robot->robot_dof(), goalCallback);
        ros::Subscriber human_state_sub = nh.subscribe("human_state", 42, humanStateCallback);
        ros::Publisher j1_pub = nh.advertise<std_msgs::Float64>(j1_topic, 1);
        ros::Publisher j2_pub = nh.advertise<std_msgs::Float64>(j2_topic, 1);
        ros::Publisher j3_pub = nh.advertise<std_msgs::Float64>(j3_topic, 1);
        ros::Publisher j4_pub = nh.advertise<std_msgs::Float64>(j4_topic, 1);
        ros::Publisher j5_pub = nh.advertise<std_msgs::Float64>(j5_topic, 1);
        ros::Publisher j6_pub = nh.advertise<std_msgs::Float64>(j6_topic, 1);
        std_msgs::Float32MultiArray robot_state_msg;
        std_msgs::Float64 j1_msg;
        std_msgs::Float64 j2_msg;
        std_msgs::Float64 j3_msg;
        std_msgs::Float64 j4_msg;
        std_msgs::Float64 j5_msg;
        std_msgs::Float64 j6_msg; 
        
        
        human_cap = robot->human_cap();
        if(use_robot)
        {    
            robot_connection->Setup(robot_ip);
            robot_connection->SendInitPack();
            recv_packet = robot_connection->Recv();
        }     
        if(nominal_mode.compare("pid") == 0)
        {
            jerk_ref = robot->pid(controller_goal);
        }
        else
        {
            jerk_ref = robot->jpc(controller_goal);
        }
        q = robot->step(jerk_ref, controller_goal);

        while(ros::ok)
        {
            // Update controller speed
            if(new_jpc_travel_time != jpc_travel_time && new_jpc_travel_time > 0)
            {  
                robot->set_JPC_speed(new_jpc_travel_time);
                jpc_travel_time = new_jpc_travel_time;
                  
                ROS_INFO_STREAM("Update jpc time");
                ROS_INFO_STREAM(jpc_travel_time);
            }

            // Get robot status
            robot_state_msg.data.clear();
            cur_q = robot->robot_q();
            cur_qd = robot->robot_qd();
            cur_qdd = robot->robot_qdd();
            
            robot->set_human_cap(human_cap);
            
            for(int j=0; j<robot->robot_dof(); j++)
            {
                robot_state_msg.data.push_back(cur_q(j));
                robot_state_msg.data.push_back(cur_qd(j));
                robot_state_msg.data.push_back(cur_qdd(j));
            }
            robot_state_pub.publish(robot_state_msg);

            // Calculate control command
            if(nominal_mode.compare("pid") == 0)
            {
                jerk_ref = robot->pid(controller_goal);
            }
            else if(nominal_mode.compare("pid_dq") == 0)
            {
                jerk_ref = robot->pid_dq(controller_goal);
            }
            else if(nominal_mode.compare("pid_vel") == 0)
            {
                jerk_ref = robot->pid_vel(controller_goal);
            }
            else
            {
                jerk_ref = robot->jpc(controller_goal);
            }
            if(ssa_enb)
            {
                jerk_safe = robot->JSSA(jerk_ref);
            }
            else
            {
                jerk_safe = jerk_ref;
            }

            // Send to real robot
            if(use_robot)
            {
                robot_connection->Send(q, recv_packet.seq_no, 0, 1);
                recv_packet = robot_connection->Recv();
            }

            // Publish to simulation
            j1_msg.data = cur_q(0) / 180 * PI;
            j2_msg.data = cur_q(1) / 180 * PI;
            j3_msg.data = cur_q(2) / 180 * PI;
            j4_msg.data = cur_q(3) / 180 * PI;
            j5_msg.data = cur_q(4) / 180 * PI;
            j6_msg.data = cur_q(5) / 180 * PI;
            j1_pub.publish(j1_msg);
            j2_pub.publish(j2_msg);
            j3_pub.publish(j3_msg);
            j4_pub.publish(j4_msg);
            j5_pub.publish(j5_msg);
            j6_pub.publish(j6_msg);
            q = robot->step(jerk_safe, controller_goal);

            if(!use_robot)
            {
                usleep(7 * microsecond); // Pause <8ms to simulate the robot controller
            }
            ros::spinOnce();
        }
        if(use_robot)
        {
            robot_connection->Send(q, recv_packet.seq_no, 1, 1);
            recv_packet = robot_connection->Recv();
            robot_connection->SendEndPack();
            robot_connection->Shutdown();
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



