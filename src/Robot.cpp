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

void Robot::set_human_cap(const std::vector<stmotion_controller::math::Capsule>& human_cap)
{
    for(int i=0; i<6; i++) 
    {
        human_cap_[i] = human_cap[i];
    }
}
        
void Robot::Setup(const std::string& DH_fname, const std::string& base_fname)
{
    q_max_.resize(njoints_, 2);
    qd_max_.resize(njoints_, 1);
    qdd_max_.resize(njoints_, 1);
    qddd_max_.resize(njoints_, 1);
    q_.resize(njoints_, 1);
    qd_.resize(njoints_, 1);
    qdd_.resize(njoints_, 1);
    target_vel.resize(njoints_, 1);
    last_target_goal.resize(njoints_, 1);
    human_cap_.resize(6);

    // q_max_ << -90.0, 90.0,
    //         -80.0, 80.0,
    //         -80.0, 80.0,
    //         -80.0, 80.0,
    //         -90.0, 90.0,
    //         -170.0, 170.0;
    q_max_ << -90.0, 90.0,
            -80.0, 80.0,
            -80.0, 80.0,
            -170.0, 170.0,
            -90.0, 90.0,
            -270.0, 270.0;
    qd_max_ << 370.0, 310.0, 410.0, 550.0, 545.0, 1000;
    qdd_max_ << 770.0, 645.0, 1025.0, 2022.0, 2128.0, 1785.0;
    qddd_max_ << 3211.0, 2690.0, 5125.0, 14868.0, 16632.0, 6377.0;
    
    for(int i=0; i<njoints_; i++)
    {
        q_.row(i) << 0.0;
        qd_.row(i) << 0.0;
        qdd_.row(i) << 0.0;
        target_vel.row(i) << 0.0;
        last_target_goal.row(i) << 0.0;
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
        if((abs(goal(idx) - q_(idx)) > 1 && pid_threshold_step_(idx) < 0) || abs(goal_(idx) - goal(idx)) > 5 || pid_threshold_step_(idx) >= 0)
        {   
            if((pid_threshold_step_(idx) < 0)  || abs(goal_(idx) - goal(idx)) > 5)
            {
                if(idx == 0)
                {
                    ROS_INFO_STREAM("Replan");
                    ROS_INFO_STREAM(q_.transpose());
                    ROS_INFO_STREAM(goal.transpose());
                    ROS_INFO_STREAM(goal_.transpose());
                }
                
                X << q_(idx), qd_(idx), qdd_(idx);
                G << goal(idx), 0, 0;
                goal_ = goal;
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

math::VectorJd Robot::pid_dq(const math::VectorJd& goal)
{
    Eigen::MatrixXd cart_T_cur = math::FK(q_, DH_, base_frame_, false);
    Eigen::MatrixXd cart_T_goal = math::FK(goal, DH_, base_frame_, false);
    Eigen::MatrixXd cart_T_goal_last = math::FK(last_target_goal, DH_, base_frame_, false);
    last_target_goal = goal;
    math::VectorJd vel = Eigen::MatrixXd::Zero(6, 1);
    math::VectorJd acc = Eigen::MatrixXd::Zero(6, 1);
    math::VectorJd jerk = Eigen::MatrixXd::Zero(6, 1);

    // calcualte the cartesian difference d_x
    Eigen::Matrix3d rot_cur = cart_T_cur.block(0, 0, 3, 3);
    Eigen::MatrixXd pos_cur = cart_T_cur.block(0, 3, 3, 1);
    Eigen::Quaterniond quat_cur(rot_cur);
    
    Eigen::Matrix3d rot_goal = cart_T_goal.block(0, 0, 3, 3);
    Eigen::MatrixXd pos_goal = cart_T_goal.block(0, 3, 3, 1);
    Eigen::Quaterniond quat_goal(rot_goal);

    Eigen::Matrix3d rot_goal_last = cart_T_goal_last.block(0, 0, 3, 3);
    Eigen::MatrixXd pos_goal_last = cart_T_goal_last.block(0, 3, 3, 1);
    Eigen::Quaterniond quat_goal_last(rot_goal_last);

    math::Vector6d d_x = math::get_6d_error(pos_cur, quat_cur, pos_goal, quat_goal);
    math::Vector6d d_x_last = math::get_6d_error(pos_goal_last, quat_goal_last, pos_goal, quat_goal);
    
    double alpha_target_vel = 0.7;
    target_vel = alpha_target_vel * d_x_last + (1 - alpha_target_vel) * target_vel;

    // calculate the jacobian of the current q
    Eigen::MatrixXd J, J_inv, qd_cmd;
    J = math::Jacobian_full(q_, DH_, base_frame_, 0);
    J_inv = math::PInv(J);
    qd_cmd = J_inv * (target_vel + 2 * d_x);
    // apply PID control to track the target d_th and get the desired jerk

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
    
    for(int idx=0; idx<njoints_; idx++)
    {
        if(abs(goal(idx) - q_(idx)) < pos_epsilon_)
        {
            resetPID(idx);
        }
    }


    for(int idx=0; idx<njoints_; idx++)
    {
        vel(idx) =  40*qd_cmd(idx);
        // max_vel_rate = std::max(max_vel_rate, std::abs(vel(idx) / qd_max_(idx)));
    }
    // if(max_vel_rate > 1.0)
    // {
    //     vel = vel / max_vel_rate;
    // }

    for(int idx=0; idx<njoints_; idx++)
    {
        err_vel = vel(idx) - qd_(idx);

        // acc(idx) = err_vel / delta_t_;

        vel_err_sum_(idx) += err_vel;
        if(last_vel_err_(idx) == 0)
        {
            last_vel_err_(idx) = err_vel;
        }
        acc(idx) = p_vel * err_vel + i_vel * vel_err_sum_(idx) * delta_t_ + d_vel * (err_vel - last_vel_err_(idx)) / delta_t_;
        last_vel_err_(idx) = err_vel;
        
        // max_acc_rate = std::max(max_acc_rate, std::abs(acc(idx) / qdd_max_(idx)));
    }

    // if(max_acc_rate > 1.0)
    // {
    //     acc = acc / max_acc_rate;
    // }
    
    for(int idx=0; idx<njoints_; idx++)
    {
        err_acc = acc(idx) - qdd_(idx);
        
        // jerk(idx) = err_acc / delta_t_;

        acc_err_sum_(idx) += err_acc;
        if(last_acc_err_(idx) == 0)
        {
            last_acc_err_(idx) = err_acc;
        }
        jerk(idx) = p_acc * err_acc + i_acc * acc_err_sum_(idx) * delta_t_ + d_acc * (err_acc - last_acc_err_(idx)) / delta_t_;
        last_acc_err_(idx) = err_acc;

        // max_jerk_rate = std::max(max_jerk_rate, std::abs(jerk(idx) / qddd_max_(idx)));
        jerk(idx) = std::min(std::max(jerk(idx), -qddd_max_(idx)), qddd_max_(idx));
    }
    // if(max_jerk_rate > 1.0)
    // {
    //     jerk = jerk / max_jerk_rate;
    // }
    
    ROS_INFO_STREAM("!!!!!!!!!!!!!!!");
    ROS_INFO_STREAM(jerk);
    return jerk;

}


math::VectorJd Robot::pid_vel(math::VectorJd& goal)
{
    for(int i=0; i<njoints_; i++)
    {
        goal(i) = std::min(std::max(goal(i), q_max_(i, 0)), q_max_(i, 1));
    }
    // if(abs(goal(4)) < 10)
    // {
    //     goal(5) = goal(5) + goal(3) - q_(3);
    //     goal(3) = q_(3);
    //     // sleep(2);
    //     ROS_INFO_STREAM("Less 10");
    //     ROS_INFO_STREAM(goal);
    //     // exit(0);
    // }

    Eigen::MatrixXd cart_T_cur = math::FK(q_, DH_, base_frame_, false);
    Eigen::MatrixXd cart_T_goal = math::FK(goal, DH_, base_frame_, false);
    Eigen::MatrixXd cart_T_goal_last = math::FK(last_target_goal, DH_, base_frame_, false);
    last_target_goal = goal;
    math::VectorJd vel = Eigen::MatrixXd::Zero(6, 1);
    math::VectorJd acc = Eigen::MatrixXd::Zero(6, 1);
    math::VectorJd jerk = Eigen::MatrixXd::Zero(6, 1);

    // calcualte the cartesian difference d_x
    Eigen::Matrix3d rot_cur = cart_T_cur.block(0, 0, 3, 3);
    Eigen::MatrixXd pos_cur = cart_T_cur.block(0, 3, 3, 1);
    Eigen::Quaterniond quat_cur(rot_cur);
    
    Eigen::Matrix3d rot_goal = cart_T_goal.block(0, 0, 3, 3);
    Eigen::MatrixXd pos_goal = cart_T_goal.block(0, 3, 3, 1);
    Eigen::Quaterniond quat_goal(rot_goal);

    Eigen::Matrix3d rot_goal_last = cart_T_goal_last.block(0, 0, 3, 3);
    Eigen::MatrixXd pos_goal_last = cart_T_goal_last.block(0, 3, 3, 1);
    Eigen::Quaterniond quat_goal_last(rot_goal_last);

    math::Vector6d d_x = math::get_6d_error(pos_cur, quat_cur, pos_goal, quat_goal);
    math::Vector6d d_x_last = math::get_6d_error(pos_goal_last, quat_goal_last, pos_goal, quat_goal);
    
    double alpha_target_vel = 0.7;
    target_vel = alpha_target_vel * d_x_last + (1 - alpha_target_vel) * target_vel;

    // calculate the jacobian of the current q
    double ori_vel_scale = 2.0;
    d_x(3) = d_x(3) * ori_vel_scale;
    d_x(4) = d_x(4) * ori_vel_scale;
    d_x(5) = d_x(5) * ori_vel_scale;

    Eigen::MatrixXd J, J_inv, qd_cmd;
    Eigen::MatrixXd goal_vel_q, q_goal;
    J = math::Jacobian_full(q_, DH_, base_frame_, 0);
    J_inv = math::PInv(J);

    d_x = d_x / 1.0;
    
    // ROS_INFO_STREAM(d_x);//target_vel + 2 * d_x);
    double determinant = J.determinant();
    ROS_INFO_STREAM(determinant);
    if(abs(determinant) < 0.07)
    {
        // if(abs(goal(4)) < 10)
        // {
        //     goal(5) = goal(5) + goal(3) - q_(3);
        //     goal(3) = q_(3);
        //     // sleep(2);
        //     // ROS_INFO_STREAM("Less 10");
        //     // ROS_INFO_STREAM(goal);
        //     // exit(0);
        // }
        qd_cmd = (goal - q_);
    }
    else
    {
        qd_cmd = J_inv * d_x;//(target_vel + 2 * d_x);
        for(int i=0; i<njoints_; i++)
        {
            qd_cmd(i) = qd_cmd(i) * 180 / PI;
        }
    }
    // if(lu_decomp.rank() < 6)
    // {
    //     ROS_INFO_STREAM("Jacob");
    //     ROS_INFO_STREAM(J);
    //     ROS_INFO_STREAM(goal);
    //     ROS_INFO_STREAM("Vel:");
    //     ROS_INFO_STREAM(goal_vel_q);
    // }
    
    goal_vel_q = qd_cmd;//J_inv * (goal_vel);
    // ROS_INFO_STREAM(goal_vel_q);
    
    // ROS_INFO_STREAM(goal_vel);
    for(int i=0; i<njoints_; i++)
    {
        goal_vel_q(i) = std::min(std::max(goal_vel_q(i), -180.0), 180.0);
    }
    // ROS_INFO_STREAM("vel");
    // ROS_INFO_STREAM(goal_vel_q);

    // ROS_INFO_STREAM(q_);
    q_goal = q_ + (goal_vel_q) * delta_t_;
    // goal_ = q_goal;

    for(int i=0; i<njoints_; i++)
    {
        q_goal(i) = std::min(std::max(q_goal(i), q_max_(i, 0)), q_max_(i, 1));
    }
    // ROS_INFO_STREAM(q_goal);
    
    double p_pos = 150;
    double i_pos = 0;
    double d_pos = 0.99;
    double p_vel = 10;
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
    jerk = Eigen::MatrixXd::Zero(6, 1);

    for(int idx=0; idx<njoints_; idx++)
    {
        err_pos = q_goal(idx) - q_(idx);
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

        if(idx == 1)
        {
            ROS_INFO_STREAM(err_pos);
            ROS_INFO_STREAM(err_vel);
            ROS_INFO_STREAM(err_acc);
            // ROS_INFO_STREAM(jerk(idx));
        }
        
        jerk(idx) = std::min(std::max(jerk(idx), -qddd_max_(idx)), qddd_max_(idx));
    }
    // ROS_INFO_STREAM(jerk);
    // ROS_INFO_STREAM("\n");
    ROS_INFO_STREAM("JERK:");
    ROS_INFO_STREAM(jerk);
    return jerk;

}


math::VectorJd Robot::step(const math::VectorJd& jerk, const math::VectorJd& goal)
{
    Eigen::MatrixXd X(3, 1);
    math::VectorJd pos_out = jerk;
    math::VectorJd jerk_clipped = jerk;
    Eigen::MatrixXd unew(3, 1);
    for(int i=0; i<6; i++)
    {
        if(pid_threshold_step_(i) == pid_thresholding_len_ - 1)
        {
            pos_out(i) = pos_out(i) * 0;
            qd_(i) = 0;
            qdd_(i) = 0;
            pid_threshold_step_(i) = -1;
        }
        if(0)//pid_threshold_step_(i) == pid_thresholding_len_)
        {
            // pos_out(i) = goal_(i);
            q_(i) = q_(i);
            qd_(i) = 0;
            qdd_(i) = 0;
            pid_threshold_step_(i) = -1;
        }
        else
        {
            X << q_(i), qd_(i), qdd_(i);
            jerk_clipped(i) = std::min(std::max(jerk(i), -qddd_max_(i)), qddd_max_(i));

            
            unew = Adt_ * X + Bdt_ * jerk_clipped(i);
            // unew(0) = std::min(std::max(unew(0), q_max_(i, 0)), q_max_(i, 1));
            // unew(1) = std::min(std::max(unew(1), -qd_max_(i)), qd_max_(i));
            // unew(2) = std::min(std::max(unew(2), -qdd_max_(i)), qdd_max_(i));

            pos_out(i) = unew(0);
            q_(i) = unew(0);
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