#pragma once

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <list>

#include <plugins/mogen_p2p_joint_wrapper.hpp>
#include <plugins/cntr_joint_imp_wrapper.hpp>

#include <eigen3/Eigen/Core>

namespace ijs_controllers{

class MoveToJointPoseStrategy{
public:
    MoveToJointPoseStrategy();
    void initialize(franka::RobotState st); 
    void trajectory (franka::RobotState st,const Eigen::Matrix<double, 7, 1> &q_g, double dq_max, double ddq_max);

    franka::Torques control(franka::RobotState st);
    void update_percept(franka::RobotState st);
    void terminate();
    bool finished();
    void context_switch(franka::RobotState st);

    std::list< Eigen::Matrix<double,7,1>> trajectoryProfile;
    std::list< Eigen::Matrix<double,7,1>> velProfile;
    Eigen::Matrix<double,7,7> m_mass;


public:
    bool set_goal(const Eigen::Matrix<double,7,1>& q_g, double dq_max, double ddq_max);
    void set_mass(std::array<double, 49> mass);

private:
    void initialize_cntr_joint_imp(franka::RobotState st);
    void initialize_mogen_p2p_joint();

    void input_cntr_joint_imp(franka::RobotState st);

private:
    franka::Torques m_panda_cmd;
    Eigen::Matrix<double,7,1> m_q_d;
    Eigen::Matrix<double,7,1> m_q_0;
    double m_dq_max;
    double m_ddq_max;
    bool goalSetting = false;

private:
    mogen_p2p_joint::mogen_p2p_joint m_mogen_p2p_joint;
    cntr_joint_imp::cntr_joint_imp m_cntr_joint_imp;
};

}
