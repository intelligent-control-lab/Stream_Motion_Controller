#include "Robot.hpp"

namespace stmotion_controller
{
namespace robot
{
Robot::Robot()
{
}
void Robot::print_robot_property()
{
    std::cout << robot_name_ << std::endl;
    std::cout << "Robot Pos: \n" << q_ << std::endl;
    std::cout << "DH: \n" << DH_ << std::endl;
    std::cout << "Robot Base: \n" << base_frame_ << std::endl;
    std::cout << "Capsules:" << std::endl;
    for(int i=0; i<njoints_; i++)
    {
        std::cout << "cap: " << i << std::endl;
        std::cout << cap_[i].p << std::endl;
        std::cout << cap_[i].r << "\n" << std::endl;
    }
    std::cout << "\n" << std::endl;
}

void Robot::set_DH_tool(const std::string fname)
{
    std::cout << "Load DH tool from: " << fname << std::endl;
    DH_tool_ = stmotion_controller::io::LoadMatFromFile(fname);
    tool_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_tool_(5, 2) * cos(0),
                 sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_tool_(5, 2) * sin(0),
                 0,       sin(0),         cos(0),        -DH_tool_(5, 1),
                 0,       0,              0,              1;
    tool_inv_ = math::PInv(tool_inv_);
}

void Robot::set_DH_tool_assemble(const std::string fname)
{
    std::cout << "Load DH tool for assemble from: " << fname << std::endl;
    DH_tool_assemble_ = stmotion_controller::io::LoadMatFromFile(fname);
    tool_assemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_tool_assemble_(5, 2) * cos(0),
                          sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_tool_assemble_(5, 2) * sin(0),
                          0,       sin(0),         cos(0),        -DH_tool_assemble_(5, 1),
                          0,       0,              0,              1;
    tool_assemble_inv_ = math::PInv(tool_assemble_inv_);
}

void Robot::set_DH_tool_disassemble(const std::string fname)
{
    std::cout << "Load DH tool for disassemble from: " << fname << std::endl;
    DH_tool_disassemble_ = stmotion_controller::io::LoadMatFromFile(fname);
    tool_disassemble_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_tool_disassemble_(5, 2) * cos(0),
                             sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_tool_disassemble_(5, 2) * sin(0),
                             0,       sin(0),         cos(0),        -DH_tool_disassemble_(5, 1),
                             0,       0,              0,              1;
    tool_disassemble_inv_ = math::PInv(tool_disassemble_inv_);

}
        
void Robot::Setup(const std::string& DH_fname, const std::string& base_fname)
{
    thetamax_.resize(njoints_, 2);
    thetamax_rad_.resize(njoints_, 2);
    thetadotmax_.resize(njoints_, 1);
    q_.resize(njoints_, 1);
    qd_.resize(njoints_, 1);
    qdd_.resize(njoints_, 1);
    
    thetamax_ << -120, 120,
                 -120, 90,
                 -90, 90,
                 -180, 180,
                 -120, 120,
                 -360, 360;
    thetadotmax_ << 370, 310, 410, 550, 545, 1000;
    for(int i=0; i<njoints_; i++)
    {
        q_.row(i) << 0.0;
        qd_.row(i) << 0.0;
        qdd_.row(i) << 0.0;
        thetamax_rad_.row(i) << thetamax_(i, 0) / 180 * PI, thetamax_(i, 1) / 180 * PI;
    }
    std::cout << "Load DH from: " << DH_fname << std::endl;
    DH_ = stmotion_controller::io::LoadMatFromFile(DH_fname);
    
    std::cout << "Load Robot Base from: " << base_fname << std::endl;
    base_frame_ = stmotion_controller::io::LoadMatFromFile(base_fname);
    T_base_inv_ = Eigen::Matrix4d::Identity(4, 4);
    T_base_inv_.col(3) << base_frame_, 1;
    T_base_inv_ = math::PInv(T_base_inv_);
    ee_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    ee_inv_ << cos(0), -sin(0)*cos(0),  sin(0)*sin(0),  DH_(5, 2) * cos(0),
               sin(0),  cos(0)*cos(0), -cos(0)*sin(0),  DH_(5, 2) * sin(0),
               0,       sin(0),         cos(0),        -DH_(5, 1),
               0,       0,              0,              1;
    ee_inv_ = math::PInv(ee_inv_);

    tool_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    tool_assemble_inv_ = Eigen::Matrix4d::Identity(4, 4); 
    tool_disassemble_inv_ = Eigen::Matrix4d::Identity(4, 4);
    
    for(int i=0; i<8; i++)
    {
        M_[i].resize(4, 4);
    }
    M_[0].col(0) << 1, 0, 0, 0;
    M_[0].col(1) << 0, 1, 0, 0;
    M_[0].col(2) << 0, 0, 1, 0;
    M_[0].col(3) << base_frame_, 1;

    cap_[0].p.resize(3, 2);
    cap_[0].p.col(0) << 0, 0, 0;
    cap_[0].p.col(1) << 0, 0, 0;
    cap_[0].r = 0.01;

    cap_[1].p.resize(3, 2);
    cap_[1].p.col(0) << -0.4, 0, 0;
    cap_[1].p.col(1) << 0, 0, 0;
    cap_[1].r = 0.13;

    cap_[2].p.resize(3, 2);
    cap_[2].p.col(0) << -0.03, 0, 0.05;
    cap_[2].p.col(1) << -0.03, 0, 0.05;
    cap_[2].r = 0.01;

    cap_[3].p.resize(3, 2);
    cap_[3].p.col(0) << 0, 0, 0;
    cap_[3].p.col(1) << 0, 0.4, 0;
    cap_[3].r = 0.068;

    cap_[4].p.resize(3, 2);
    cap_[4].p.col(0) << 0, 0, 0.01;
    cap_[4].p.col(1) << 0, 0, -0.2;
    cap_[4].r = 0.1;

    cap_[5].p.resize(3, 2);
    cap_[5].p.col(0) << 0.05, 0, 0.1107;
    cap_[5].p.col(1) << 0.18, 0, 0.1107;
    cap_[5].r = 0.01;

    cap_cur_[0].p.resize(3, 2);
    cap_cur_[0].r = cap_[0].r;
    cap_cur_[1].p.resize(3, 2);
    cap_cur_[1].r = cap_[1].r;
    cap_cur_[2].p.resize(3, 2);
    cap_cur_[2].r = cap_[2].r;
    cap_cur_[3].p.resize(3, 2);
    cap_cur_[3].r = cap_[3].r;
    cap_cur_[4].p.resize(3, 2);
    cap_cur_[4].r = cap_[4].r;
    cap_cur_[5].p.resize(3, 2);
    cap_cur_[5].r = cap_[5].r;

    human_cap_[0].p.resize(3, 2);
    human_cap_[0].r = 0.0;
    human_cap_[0].p.col(0) << 1.21, 0, 1.7;
    human_cap_[0].p.col(1) << 1.2, 0, 1.71;
    human_cap_[1].p.resize(3, 2);
    human_cap_[1].r = 0.1;
    human_cap_[1].p.col(0) << 10, 0, 0;
    human_cap_[1].p.col(1) << 10, 0, 0;
    human_cap_[2].p.resize(3, 2);
    human_cap_[2].r = 0.0;
    human_cap_[2].p.col(0) << 10, 0, 0;
    human_cap_[2].p.col(1) << 10, 0, 0;
    human_cap_[3].p.resize(3, 2);
    human_cap_[3].r = 0.0;
    human_cap_[3].p.col(0) << 10, 0, 0;
    human_cap_[3].p.col(1) << 10, 0, 0;
    human_cap_[4].p.resize(3, 2);
    human_cap_[4].r = 0.0;
    human_cap_[4].p.col(0) << 10, 0, 0;
    human_cap_[4].p.col(1) << 10, 0, 0;
    human_cap_[5].p.resize(3, 2);
    human_cap_[5].r = 0.0;
    human_cap_[5].p.col(0) << 10, 0, 0;
    human_cap_[5].p.col(1) << 10, 0, 0;

    critical_pts_ = Eigen::MatrixXd::Zero(3, 2);
    critical_pts_tmp_ = Eigen::MatrixXd::Zero(3, 2);
    J_.resize(3, njoints_);
    Jd_.resize(3, njoints_);
    Jdd_.resize(3, njoints_);
    for(int i=0; i<6; i++)
    {
        J_full_[i].resize(3, njoints_);
        Jd_full_[i].resize(3, njoints_);
    }
    state_.resize(9, 1);
    Am_.resize(9, 9);
    Bm_.resize(9, 3);
    Am_ << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Identity(3, 3) * delta_t_, Eigen::MatrixXd::Identity(3, 3) * 0.5 * pow(delta_t_, 2),
           Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Identity(3, 3) * delta_t_,
           Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3);
    Bm_ << Eigen::MatrixXd::Identity(3, 3) * pow(delta_t_, 3) / 6, 
           Eigen::MatrixXd::Identity(3, 3) * 0.5 * pow(delta_t_, 2), 
           Eigen::MatrixXd::Identity(3, 3) * delta_t_;

    P1_.resize(9, 9);
    P2_.resize(9, 9);
    P3_.resize(9, 9);
    P4_.resize(9, 9);
    P1_ << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(6, 9);
    P2_ << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(6, 9);
    P3_ << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(6, 9);
    P4_ << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), 
           Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 9);
    Q_ = Eigen::MatrixXd::Identity(njoints_, njoints_);
    Adt_.resize(3, 3);
    Bdt_.resize(3, 1);
    Adt_ << 1, delta_t_, 0.5 * pow(delta_t_, 2), 
            0, 1, delta_t_, 
            0, 0, 1;
    Bdt_ << pow(delta_t_, 3) / 6.0, 0.5 * pow(delta_t_, 2), delta_t_;

    pos_err_sum_ = Eigen::MatrixXd::Zero(6, 1);
    vel_err_sum_ = Eigen::MatrixXd::Zero(6, 1);
    acc_err_sum_ = Eigen::MatrixXd::Zero(6, 1);
    last_pos_err_ = Eigen::MatrixXd::Zero(6, 1);
    last_vel_err_ = Eigen::MatrixXd::Zero(6, 1);
    last_acc_err_ = Eigen::MatrixXd::Zero(6, 1);
    pid_threshold_step_ = Eigen::MatrixXd::Constant(njoints_, 1, -1);
    pid_thresholding_ubuffer_ = Eigen::MatrixXd::Zero(njoints_, pid_thresholding_len_);
    pid_A_ = Eigen::MatrixXd::Identity(3, 3);
    pid_B_ = Eigen::MatrixXd::Zero(3, pid_thresholding_len_);

    for(int i=0; i<pid_thresholding_len_; i++)
    {
        pid_B_.col(pid_thresholding_len_ - i - 1) << pid_A_ * Bdt_;
        pid_A_ = Adt_ * pid_A_;
    }
    pid_Binv_ = math::PInv(pid_B_);

    std::cout << std::setprecision(5);
    std::cout << "Robot Setup Done!" << std::endl;
}

void Robot::set_JPC_speed(const double& t)
{
    pid_thresholding_len_ = round(t * 125);
    pid_thresholding_ubuffer_ = Eigen::MatrixXd::Zero(njoints_, pid_thresholding_len_);
    pid_A_ = Eigen::MatrixXd::Identity(3, 3);
    pid_B_ = Eigen::MatrixXd::Zero(3, pid_thresholding_len_);

    for(int i=0; i<pid_thresholding_len_; i++)
    {
        pid_B_.col(pid_thresholding_len_ - i - 1) << pid_A_ * Bdt_;
        pid_A_ = Adt_ * pid_A_;
    }
    pid_Binv_ = math::PInv(pid_B_);
    
    for(int i=0; i<njoints_; i++)
    {
        pid_threshold_step_(i) = -1;
    }
}


math::VectorJd Robot::IK(const math::VectorJd& cur_q, const Eigen::Matrix4d& goal_T, const Eigen::MatrixXd& DH,
                         const Eigen::Matrix4d& T_tool_inv, const bool& joint_rad, bool& status)
{
    double eps = 1e-10;
    status = false;
    math::VectorJd theta = cur_q;
    Eigen::MatrixXd DH_cur = DH;
    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<cur_q.rows(); i++)
        {
            theta(i) = theta(i) * PI / 180;
        }
    }
    math::VectorJd cur_theta = theta;
    math::VectorJd theta_tmp = theta;
    Eigen::Matrix4d T = T_base_inv_ * goal_T * T_tool_inv;
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> P = T.block(0, 3, 3, 1);
    double X, Y, Z, r, r2, a1, a12, a2, a22, a3, a32, d4, d42, m, e, c, l, l2, h, f1, f2, t1, t2, t3, k, g1, g2, q1, q2, min_diff;
    double th1_tmp, th2_tmp, th3_tmp, th4_tmp, th5_tmp, th6_tmp;
    X = P(0, 0);
    Y = P(1, 0);
    Z = P(2, 0);
    r = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));
    a1 = DH(0, 2);
    a2 = DH(1, 2);
    a3 = DH(2, 2);
    d4 = DH(3, 1);
    r2 = pow(r, 2);
    a12 = pow(a1, 2);
    a22 = pow(a2, 2);
    a32 = pow(a3, 2);
    d42 = pow(d4, 2);
    m = a32 + d42;
    e = 2 * r2;
    c = 4 * a12;
    l = a2 * (2 * m + 2 * a12 + 2 * a22 - c - e);
    l2 = pow(l, 2);
    h = (c + e) * m - pow((m + a12 + a22), 2) + a12 * e + a22 * e + 4 * a12 * a22 - 4 * a12 * pow(Z, 2) - pow(r, 4);

    double cond1, cond2, th2_tmp1, th2_tmp2;
    cond1 = 4 * l2 + 16 * a22 * h;
    min_diff = 10000;
    Eigen::MatrixXd th23_candidates;
    int th23_candidate_cnt, th1_candidate_cnt, all_candidate_cnt;
    th23_candidate_cnt = 0;
    th1_candidate_cnt = 0;
    all_candidate_cnt = 0;
    th23_candidates.resize(8, 2);
    if(cond1 >= 0)
    {
        f1 = (-2 * l + sqrt(cond1)) / (8 * a22 + eps);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2 >= 0)
        {
            // First candidate
            th3_tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1 + eps));

            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;

            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }

            // Second candidate
            th3_tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
        }
        f1 = (-2 * l - sqrt(cond1)) / (8 * a22 + eps);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2)
        {
            // Third candidate
            th3_tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
            
            // Fourth candidate
            th3_tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
        }
    }
    else
    {
        status = false;
        return cur_q;
    }
    
    Eigen::MatrixXd th1_candidates, candidates;
    Eigen::Matrix4d verify_T;
    th1_candidates.resize(2, 1);
    candidates.resize(32, njoints_);
    double th1_tmp1, th1_tmp2;
    for(int i=0; i<th23_candidate_cnt; i++)
    {
        th2_tmp = th23_candidates(i, 0) - PI / 2;
        th3_tmp = th23_candidates(i, 1);
        th1_candidate_cnt = 0;

        g1 = f1 * cos(th2_tmp) + f2 * sin(th2_tmp) + a2 * cos(th2_tmp);
        g2 = f1 * sin(th2_tmp) - f2 * cos(th2_tmp) + a2 * sin(th2_tmp);
        q1 = g1+a1;
        q2 = 0;
        cond1 = pow(q2, 2) + pow(q1, 2) - pow(X, 2);
        q1 = 0;
        q2 = g1+a1;
        th1_tmp1 = 2 * atan((q2 + sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y + eps));
        th1_tmp2 = 2 * atan((q2 - sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y + eps));
        if(th1_tmp1 < thetamax_rad_(0, 1) && th1_tmp1 > thetamax_rad_(0, 0))
        {
            th1_candidates.row(th1_candidate_cnt) << th1_tmp1;
            th1_candidate_cnt ++;
        }
        if(th1_tmp2 < thetamax_rad_(0, 1) && th1_tmp2 > thetamax_rad_(0, 0))
        {
            th1_candidates.row(th1_candidate_cnt) << th1_tmp2;
            th1_candidate_cnt ++;
        }
        for(int j=0; j<th1_candidate_cnt; j++)
        {
            theta_tmp(0) = th1_candidates(j, 0);
            theta_tmp(1) = th2_tmp + PI / 2;;
            theta_tmp(2) = th3_tmp;
            DH_cur.col(0) = DH.col(0) + theta_tmp;
            Eigen::Matrix3d R03 = Eigen::Matrix3d::Identity(3, 3);
            Eigen::MatrixXd a = DH_cur.col(3);
            Eigen::MatrixXd q = DH_cur.col(0);
            Eigen::Matrix3d temp(3, 3); 
            for(int k=0; k<3; k++)
            {
                temp << cos(q(k)), -sin(q(k)) * cos(a(k)),  sin(q(k)) * sin(a(k)),  
                        sin(q(k)),  cos(q(k)) * cos(a(k)), -cos(q(k)) * sin(a(k)),  
                        0,          sin(a(k)),              cos(a(k));
                R03 = R03 * temp;
            }
            Eigen::Matrix3d R36 = math::PInv(R03) * R;
            th5_tmp = acos(-R36(2, 2));
            double s5 = sin(th5_tmp) + eps;

            if(abs(s5) <= 0.000001)
            {
                th4_tmp = 0;
                th5_tmp = 0;
                th6_tmp = atan2(R36(0, 1), R36(0, 0));
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }

                th5_tmp = PI;
                th6_tmp = atan2(R36(1, 0), -R36(1, 1));
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
            }
            else
            {
                double th4_1 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                double th6_1 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
                double sum1 = sqrt(pow(th5_tmp, 2) + pow(th4_1, 2) + pow(th6_1, 2));
                s5 = sin(-th5_tmp);
                double th4_2 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                double th6_2 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
                double sum2 = sqrt(pow(th5_tmp, 2) + pow(th4_2, 2) + pow(th6_2, 2));

                th4_tmp = th4_1;
                th6_tmp = th6_1;
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
                
                th5_tmp = -th5_tmp;
                th4_tmp = th4_2;
                th6_tmp = th6_2;
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
            } 
        }  
    }
    status = false;
    for(int i=0; i<all_candidate_cnt; i++)
    {   
        theta_tmp = candidates.row(i);
        verify_T = math::FK(theta_tmp, DH, base_frame_, 1);
        if(verify_T.isApprox(goal_T, 0.01) && (theta_tmp - cur_theta).norm() < min_diff)// && joint_in_range(theta_tmp, 1))
        {
            theta = theta_tmp;
            min_diff = (theta_tmp - cur_theta).norm();
            status = true;            
        }
    }
    if(!status)
    {
        return cur_q;
    }

    // Rad to Deg
    for(int i=0; i<theta.rows(); i++)
    {
        theta(i) = theta(i) * 180 / PI;
    }
    return theta;
}

bool Robot::joint_in_range(const math::VectorJd& theta, const bool& is_rad)
{
    math::VectorJd theta_deg = theta;
    if(is_rad)
    {
        // Rad to Deg
        for(int i=0; i<theta.rows(); i++)
        {
            theta_deg(i) = theta_deg(i) / PI * 180;
        }
    }
    for(int i=0; i<theta.rows(); i++)
    {
        if(theta_deg(i) < thetamax_(i, 0) || theta_deg(i) > thetamax_(i, 1))
        {
            return false;
        }
    }
    return true;
}

math::VectorJd Robot::JSSA(const math::VectorJd& jerk_ref)
{
    math::VectorJd jerk_ref_rad = jerk_ref;
    math::VectorJd jerk_out = jerk_ref;
    math::VectorJd pos_out = jerk_ref;
    Eigen::MatrixXd q_rad(6, 1), qd_rad(6, 1), qdd_rad(6, 1), DH_cur(6, 4);
    DH_cur = DH_;

    // Deg to Rad
    for(int i=0; i<njoints_; i++)
    {
        jerk_ref_rad(i) = jerk_ref(i) * PI / 180;
        q_rad(i) = q_(i) * PI / 180;
        qd_rad(i) = qd_(i) * PI / 180;
        qdd_rad(i) = qdd_(i) * PI / 180;
    }
    DH_cur.col(0) = DH_.col(0) + q_rad;

    // Calculate current robot configuration
    Eigen::MatrixXd R(3, 3);
    Eigen::MatrixXd T(3, 1);
    Eigen::MatrixXd temp(4, 4);
    for(int i=0; i<DH_cur.rows(); i++)
    {
        R << cos(DH_cur.coeff(i, 0)), -sin(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)),  sin(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             sin(DH_cur.coeff(i, 0)),  cos(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)), -cos(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             0,                        sin(DH_cur.coeff(i, 3)),                            cos(DH_cur.coeff(i, 3));

        T << DH_cur.coeff(i, 2) * cos(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 2) * sin(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 1);
        temp << R, T, 0, 0, 0, 1;
        M_[i + 1] = M_[i] * temp;
        cap_cur_[i].p.col(0) << M_[i + 1].block(0, 0, 3, 3) * cap_[i].p.col(0) + M_[i + 1].block(0, 3, 3, 1);
        cap_cur_[i].p.col(1) << M_[i + 1].block(0, 0, 3, 3) * cap_[i].p.col(1) + M_[i + 1].block(0, 3, 3, 1);
    }

    double dist = 0.0;
    double dmin = 100.0;
    double phi_max = -1000;
    critical_link1_ = njoints_ - 1;
    critical_link2_ = 0;
    critical_pts_.col(0) << cap_cur_[critical_link1_].p.col(1);
    critical_pts_.col(1) << human_cap_[critical_link2_].p.col(1);
    Eigen::MatrixXd BJ, D;
    Eigen::MatrixXd H(9, 1);

    // Find critical points based on p2p distance
    for(int i=1; i<njoints_; i++)
    {
        if(cap_cur_[i].r > 0)
        {
            for(int j=0; j<nlink_human_; j++)
            {
                if(human_cap_[j].r > 0)
                {
                    dist = math::DistCap2Cap(cap_cur_[i], human_cap_[j], critical_pts_tmp_);
                    if(dist < dmin)
                    {
                        dmin = dist;
                        critical_link1_ = i + 1;
                        critical_pts_ << critical_pts_tmp_;
                        critical_link2_ = j;
                    }
                } 
            }
        }
    }

    // Compute Jacobian and its derivatives
    computeJacobiDiff(DH_cur, qd_rad, qdd_rad, critical_link1_, critical_pts_.col(0));
    BJ = Bm_ * J_;
    state_ << critical_pts_.col(0), J_ * qd_rad, J_ * qdd_rad + Jd_ * qd_rad;
    H << critical_pts_.col(1), Eigen::MatrixXd::Zero(6, 1);
    D = Am_ * state_ + Bm_ * (Jdd_ * qd_rad + 2 * Jd_ * qdd_rad) - H;

    // Calculate safety index
    calculateSafetyIdx(D, BJ, margin_ + cap_cur_[critical_link1_ - 1].r + human_cap_[critical_link2_].r, jerk_ref_rad);

    // Unsafe: modify jerk
    if(phi_safe_ > 0)
    {
        jerk_ref_rad = jerk_ref_rad - ((vet_ * jerk_ref_rad).coeff(0, 0) - thres_) * Q_ * vet_.transpose() / (vet_ * Q_ * vet_.transpose()).coeff(0, 0);
        pid_threshold_step_ = Eigen::MatrixXd::Constant(njoints_, 1, -1);
        // std::cout << "Unsafe. Link R: " << critical_link1_ << " Link H: " << critical_link2_ << " Dist:" << dmin << " Phi: " << phi_safe_ << std::endl;
        ssa_on_ = 1;
    }
    // Safe: do nothing
    else{
        // std::cout << "Safe. " << " Dist:" << dmin << std::endl;
        ssa_on_ = 0;
    }

    for(int i=0; i<6; i++)
    {
        jerk_out(i) = jerk_ref_rad(i) * 180 / PI;
        if(jerk_out(i) > jerk_max_(i))
        {
            jerk_out(i) = jerk_max_(i);
            pid_threshold_step_(i) = -1;
        }
        else if(jerk_out(i) < -jerk_max_(i))
        {
            jerk_out(i) = -jerk_max_(i);
            pid_threshold_step_(i) = -1;
        }
    }
    return jerk_out;
}

void Robot::computeJacobiDiff(const Eigen::MatrixXd& DH_cur, const Eigen::MatrixXd& qd, const Eigen::MatrixXd& qdd, 
                              const int& robot_critical_link_id, const Eigen::MatrixXd& critical_pt)
{
    Eigen::Matrix<double, 3, Eigen::Dynamic> z = Eigen::MatrixXd::Zero(3, njoints_);
    Eigen::Matrix<double, 3, Eigen::Dynamic> r = Eigen::MatrixXd::Zero(3, njoints_ + 1);
    Eigen::Matrix<double, 3, Eigen::Dynamic> vr = Eigen::MatrixXd::Zero(3, njoints_ + 1);
    Eigen::Matrix<double, 3, Eigen::Dynamic> ar = Eigen::MatrixXd::Zero(3, njoints_ + 1);
    Eigen::Matrix<double, 3, Eigen::Dynamic> omega = Eigen::MatrixXd::Zero(3, njoints_);
    Eigen::Matrix<double, 3, Eigen::Dynamic> alpha = Eigen::MatrixXd::Zero(3, njoints_);
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T.col(3) << base_frame_, 1;

    Eigen::MatrixXd a = DH_cur.col(3);
    Eigen::MatrixXd A = DH_cur.col(2);
    Eigen::MatrixXd D = DH_cur.col(1);
    Eigen::MatrixXd q = DH_cur.col(0);
    Eigen::MatrixXd temp(4, 4);

    for(int i=0; i<njoints_; i++)
    {
        z.col(i) << T.block(0, 2, 3, 1);
        r.col(i) << T.block(0, 3, 3, 1);
        if(i > 0)
        {
            omega.col(i) << omega.col(i - 1) + z.col(i) * qd(i);
            alpha.col(i) << alpha.col(i - 1) + z.col(i) * qdd(i);
        }
        else
        {
            omega.col(i) << z.col(i) * qd(i);
            alpha.col(i) << z.col(i) * qdd(i);
        }
        temp << cos(q(i)), -sin(q(i)) * cos(a(i)),  sin(q(i)) * sin(a(i)),  A(i) * cos(q(i)),
                sin(q(i)),  cos(q(i)) * cos(a(i)), -cos(q(i)) * sin(a(i)),  A(i) * sin(q(i)),
                0,          sin(a(i)),            cos(a(i)),            D(i),
                0,          0,                    0,                    1;
        T = T * temp;
    }
    r.col(njoints_) << T.block(0, 3, 3, 1);

    J_ = Eigen::MatrixXd::Zero(3, njoints_);
    Jd_ = Eigen::MatrixXd::Zero(3, njoints_);
    Jdd_ = Eigen::MatrixXd::Zero(3, njoints_);
    for(int i=0; i<6; i++)
    {
        J_full_[i] = Eigen::MatrixXd::Zero(3, njoints_);
        Jd_full_[i] = Eigen::MatrixXd::Zero(3, njoints_);;
    }
    for(int i=0; i<robot_critical_link_id; i++)
    {
        J_.col(i) = z.col(i).cross(Eigen::Vector3d{critical_pt - r.col(i)});
        for(int j=0; j<i+1; j++)
        {
            J_full_[i].col(j) << (r.col(j) - r.col(i + 1)).cross(z.col(j));
        }
    }
    Eigen::Matrix<double, 3, 1> vp = J_ * qd;
    vr.col(0) << Eigen::MatrixXd::Zero(3, 1);
    
    for(int i=1; i<njoints_; i++)
    {
        Eigen::Matrix<double, 3, Eigen::Dynamic> jacob = J_full_[i];
        vr.col(i + 1) << jacob * qd;    
    }

    for(int i=0; i<robot_critical_link_id; i++)
    {
        Jd_.col(i) << (r.col(i) - critical_pt).cross(omega.col(i).cross(z.col(i))) + (vr.col(i) - vp).cross(z.col(i));
        for(int j=0; j<i+1; j++)
        {
            Jd_full_[i].col(j) << (r.col(j) - r.col(i + 1)).cross(omega.col(j).cross(z.col(j))) + (vr.col(j) - vr.col(i + 1)).cross(z.col(j));
        }
    }

    Eigen::Matrix<double, 3, 1> ap = Jd_ * qd + J_ * qdd;
    ar.col(0) << Eigen::MatrixXd::Zero(3, 1);
    for(int i=0; i<njoints_; i++)
    {
        Eigen::MatrixXd jacob = J_full_[i];
        Eigen::MatrixXd jacob_d = Jd_full_[i];
        ar.col(i + 1) = jacob_d * qd + jacob * qdd;
    }

    for(int i=0; i<robot_critical_link_id; i++)
    {
        Eigen::Matrix<double, 3, 1> tmp = alpha.col(i).cross(z.col(i)) + omega.col(i).cross(omega.col(i).cross(z.col(i)));
        Jdd_.col(i) << tmp.cross(Eigen::Vector3d{critical_pt - r.col(i)}) 
                       + 2 * (omega.col(i).cross(z.col(i))).cross(vp - vr.col(i)) 
                       + z.col(i).cross(ap - ar.col(i));
    }
}


void Robot::calculateSafetyIdx(const Eigen::MatrixXd& D, const Eigen::MatrixXd& BJ, const double& margin, const Eigen::MatrixXd& u)
{
    Eigen::MatrixXd dm = D + BJ * u;
    Eigen::MatrixXd dm_t = dm.transpose();
    Eigen::MatrixXd D_t = D.transpose();
    double d = sqrt((dm_t * P1_ * dm).coeff(0, 0));
    double dd = (dm_t * P2_ * dm).coeff(0, 0) / d;
    double ddd = -pow(dd, 2) / d + (dm_t * P4_ * dm).coeff(0, 0) / d + (dm_t * P3_ * dm).coeff(0, 0) / d;
    double d_approx = sqrt((D_t * P1_ * D).coeff(0, 0));

    double a = (dm_t * P1_ * dm).coeff(0, 0);
    phi_safe_ = pow(margin, 2) * d - pow(d, 3) - jssa_k1_ * dd * d - jssa_k2_ * ddd * d;
    
    thres_ = pow(margin, 2) * d_approx - pow(d_approx, 3) - jssa_k1_ * (D_t * P2_ * D).coeff(0, 0) - 
             jssa_k2_ * (D_t * P3_ * D).coeff(0, 0) - jssa_k2_ * (D_t * P4_ * D).coeff(0, 0) +
             jssa_k2_ * pow((D_t * P2_ * D).coeff(0, 0), 2) / (D_t * P1_ * D).coeff(0, 0);
    vet_ = 2 * jssa_k2_ * D_t * P3_ * BJ + 2 * jssa_k1_ * D_t * P2_ * BJ + 2 * jssa_k2_ * D_t * P4_ * BJ;
}


bool Robot::is_static()
{
    for(int i=0; i<njoints_; i++)
    {
        if(abs(qd_(i)) > vel_epsilon_ || abs(qdd_(i)) > acc_epsilon_)
        {
            return false;
        }
    }
    return true;
}


bool Robot::reached_goal(math::VectorJd goal)
{
    for(int i=0; i<njoints_; i++)
    {
        if(abs(q_(i) - goal(i)) > 0.00001)
        {
            return false;
        }
    }
    return true;
}

math::VectorJd Robot::jpc(const math::VectorJd& goal)
{
    math::VectorJd jerk = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd X(3, 1);
    Eigen::MatrixXd G(3, 1);
    Eigen::MatrixXd tmp(3, 1);
    for(int idx=0; idx<njoints_; idx++)
    {
        if((goal(idx) != q_(idx) && pid_threshold_step_(idx) < 0) || pid_threshold_step_(idx) >= 0)
        {   
            if(pid_threshold_step_(idx) < 0)
            {
                X << q_(idx), qd_(idx), qdd_(idx);
                G << goal(idx), 0, 0;
                tmp = G - pid_A_ * X;
                pid_thresholding_ubuffer_.row(idx) = (pid_Binv_ * tmp).transpose();
                pid_threshold_step_(idx) = 0;
                resetPID(idx);
                jerk(idx) = pid_thresholding_ubuffer_.coeff(idx, pid_threshold_step_(idx));
            }
            else if(pid_threshold_step_(idx) < pid_thresholding_len_ - 1)
            {
                pid_threshold_step_(idx) ++;
                jerk(idx) = pid_thresholding_ubuffer_.coeff(idx, pid_threshold_step_(idx));
            }
            else
            {
                pid_threshold_step_(idx) ++;
                jerk(idx) = 0;
            }
        }
    }
    return jerk;
}

math::VectorJd Robot::pid(const math::VectorJd& goal)
{
    double p_pos = 3;
    double i_pos = 0;
    double d_pos = 0.99;
    double p_vel = 2.1;
    double i_vel = 0;
    double d_vel = 0.1;
    double p_acc = 10;
    double i_acc = 0;
    double d_acc = 0.5;
    double err_pos = 0;
    double err_vel = 0;
    double err_acc = 0;
    double vel_ref = 0;
    double acc_ref = 0;
    math::VectorJd jerk = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd X(3, 1);
    Eigen::MatrixXd G(3, 1);
    Eigen::MatrixXd tmp(3, 1);
    for(int idx=0; idx<njoints_; idx++)
    {
        if((abs(goal(idx) - q_(idx)) < pos_epsilon_ && 
            goal(idx) != q_(idx) && pid_threshold_step_(idx) < 0) || pid_threshold_step_(idx) >= 0)
        {   
            if(pid_threshold_step_(idx) < 0)
            {
                X << q_(idx), qd_(idx), qdd_(idx);
                G << goal(idx), 0, 0;
                tmp = G - pid_A_ * X;
                pid_thresholding_ubuffer_.row(idx) = (pid_Binv_ * tmp).transpose();
                pid_threshold_step_(idx) = 0;
                resetPID(idx);
                jerk(idx) = pid_thresholding_ubuffer_.coeff(idx, pid_threshold_step_(idx));
            }
            else if(pid_threshold_step_(idx) < pid_thresholding_len_ - 1)
            {
                pid_threshold_step_(idx) ++;
                jerk(idx) = pid_thresholding_ubuffer_.coeff(idx, pid_threshold_step_(idx));
            }
            else
            {
                pid_threshold_step_(idx) ++;
                jerk(idx) = 0;
            }
        }
        // Cascade PID
        else
        {
            err_pos = goal(idx) - q_(idx);
            pos_err_sum_(idx) += err_pos;
            if(last_pos_err_(idx) == 0)
            {
                last_pos_err_(idx) = err_pos;
            }
            vel_ref = p_pos * err_pos + i_pos * pos_err_sum_(idx) * delta_t_ + d_pos * (err_pos - last_pos_err_(idx)) / delta_t_;
            last_pos_err_(idx) = err_pos;

            err_vel = vel_ref - qd_(idx);
            vel_err_sum_(idx) += err_vel;
            if(last_vel_err_(idx) == 0)
            {
                last_vel_err_(idx) = err_vel;
            }
            acc_ref = p_vel * err_vel + i_vel * vel_err_sum_(idx) * delta_t_ + d_vel * (err_vel - last_vel_err_(idx)) / delta_t_;
            last_vel_err_(idx) = err_vel;

            err_acc = acc_ref - qdd_(idx);
            acc_err_sum_(idx) += err_acc;
            if(last_acc_err_(idx) == 0)
            {
                last_acc_err_(idx) = err_acc;
            }
            jerk(idx) = p_acc * err_acc + i_acc * acc_err_sum_(idx) * delta_t_ + d_acc * (err_acc - last_acc_err_(idx)) / delta_t_;
            last_acc_err_(idx) = err_acc;
        }
    }
    return jerk;
}


math::VectorJd Robot::step(const math::VectorJd& jerk, const math::VectorJd& goal)
{
    Eigen::MatrixXd X(3, 1);
    math::VectorJd pos_out = jerk;
    Eigen::MatrixXd unew(3, 1);
    for(int i=0; i<6; i++)
    {
        if(pid_threshold_step_(i) == pid_thresholding_len_ - 1)
        {
            pos_out(i) = goal(i);
            q_(i) = goal(i);
            qd_(i) = 0;
            qdd_(i) = 0;
            pid_threshold_step_(i) = -1;
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

void Robot::resetPID(int i)
{
    pos_err_sum_(i) = 0;
    vel_err_sum_(i) = 0;
    acc_err_sum_(i) = 0;
    last_pos_err_(i) = 0;
    last_vel_err_(i) = 0;
    last_acc_err_(i) = 0;
}


}
}
