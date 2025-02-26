#include "Robot.hpp"

using namespace std::chrono;

stmotion_controller::math::VectorJd robot_q = Eigen::MatrixXd::Zero(6, 1);
stmotion_controller::math::VectorJd robot_qd = Eigen::MatrixXd::Zero(6, 1);
stmotion_controller::math::VectorJd robot_qdd = Eigen::MatrixXd::Zero(6, 1);

void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}

bool reached_goal(const stmotion_controller::math::VectorJd& goal)
{
    for(int i=0; i<6; i++)
    {
        if(abs(goal(i) - robot_q(i)) > 0.01 || abs(robot_qd(i)) > 0.001 || abs(robot_qdd(i)) > 0.001)
        {
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "use_controller_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        ros::Rate loop_rate(150);

        Eigen::Matrix<double, 6, 5> joint_goal = Eigen::Matrix<double, 6, 5>::Zero();
        joint_goal.col(0) << 0, 0, 0, 0, 0, 0;
        joint_goal.col(1) << 0, 0, 0, 0, -90, 0;
        joint_goal.col(2) << 0, 0, 0, 0, 0, 0;
        joint_goal.col(3) << 0, 0, 0, 0, -90, 0;
        joint_goal.col(4) << 0, 0, 0, 0, 0, 0;
        int robot_dof = 6;
       
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("/stmotion_controller_bringup/robot_goal", robot_dof);
        ros::Publisher jpc_time_pub = nh.advertise<std_msgs::Float64>("/stmotion_controller_bringup/jpc_travel_time", 1);
        ros::Subscriber robot_state_sub = nh.subscribe("/stmotion_controller_bringup/robot_state", robot_dof * 3, robotStateCallback);
        std_msgs::Float32MultiArray goal_msg;
        std_msgs::Float64 jpc_time_msg;
        
        int num_tasks = joint_goal.cols();
        int task_idx = -1;
        stmotion_controller::math::VectorJd cur_goal = Eigen::MatrixXd::Zero(6, 1);
        cur_goal = joint_goal.col(0);
        unsigned int microsecond = 1000;
        usleep(300 * microsecond);
        while(ros::ok)
        {
            if(task_idx == -1 || reached_goal(cur_goal))
            {
                if(task_idx == -1)
                {
                    jpc_time_msg.data = 0.5;
                    jpc_time_pub.publish(jpc_time_msg);
                }
                else if (task_idx == 1)
                {
                    jpc_time_msg.data = 5;
                    jpc_time_pub.publish(jpc_time_msg);
                }
                else if(task_idx == 3)
                {
                    jpc_time_msg.data = 0.1;
                    jpc_time_pub.publish(jpc_time_msg);
                }
                task_idx ++;
                if(task_idx >= joint_goal.cols())
                {
                    break;
                }
                cur_goal = joint_goal.col(task_idx);     
            }
            goal_msg.data.clear();
            for(int j=0; j<robot_dof; j++)
            {
                goal_msg.data.push_back(cur_goal(j));
            }
            goal_pub.publish(goal_msg);
            ros::spinOnce();
        }
        ROS_INFO_STREAM("Task Execution Done!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


