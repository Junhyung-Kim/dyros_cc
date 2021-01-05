
#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include <ros/ros.h>
#include "tocabi_controller/model.h"
#include "std_msgs/String.h"
#include "math_type_define.h"

class CustomController
{
public:
    CustomController(DataContainer &dc, RobotData &rd);
    Eigen::VectorQd getControl();

    void taskCommandToCC(TaskCommand tc_);

    void computeSlow();
    void computeFast();
    void computePlanner();

    DataContainer &dc_;
    RobotData &rd_;
    WholebodyController &wbc_;
    Walking_controller &wkc_;
    TaskCommand tc;

    ros::Subscriber tocabi_pinocchio;

    const std::string FILE_NAMES[2] =
        {
            ///change this directory when you use this code on the other computer///
            "/home/jhk/data/walking/0_tocabi_.txt",
            "/home/jhk/data/walking/1_tocabi_.txt",
    };

    std::fstream file[2];

    bool command_init = false;
    std::chrono::high_resolution_clock::time_point t_begin;
    int cycle_count;

    Eigen::Vector3d xipos_prev;
    Eigen::MatrixXd foot_stepc;
    Eigen::VectorQd torque_prev;
    int contactModec;
    int walking_tickc;
    bool phaseChangec;
    bool phaseChangec1;
    int current_step_numc;
    int total_step_numc;
    int double2Single_prec;
    int double2Singlec;
    int single2Double_prec;
    int single2Doublec;
    double rate;
    double L;
    Eigen::VectorQd Int_dis;
    Eigen::VectorQd torque_dis;

    Eigen::VectorQd torque_dis_prev;
    Eigen::Vector2d zmp_prevlpf;
    Eigen::Vector4d torque_dis1_prev;
    bool callback_check = false;
    Eigen::Vector3d com_dotprev;
    Eigen::Vector3d com_prev;
   
    int callback_tick = 0;

    ros::Publisher joint_pin_pub;
    ros::Publisher dist_pub;
    sensor_msgs::JointState joint_state_msg;

    Eigen::Vector12d contactforce_lpf;
    Eigen::Vector12d contactforce_prev;

    Eigen::VectorVQd q_dot_virtual_prev;
    Eigen::VectorVQd q_ddot_virtual_;
    Eigen::VectorQd q_est1;
    Eigen::VectorQd q_dot_est1;
    Eigen::VectorQd q_dot_est1_prev;
    Eigen::VectorQd q_dot_est1_pprev;
    Eigen::VectorQd q_ddot_est1;

    Eigen::Vector3d CM_moment_pin;
    bool velEst = false;
    bool debug = false;
   // bool velEst1 = false;

    std::mutex mtx_wlk;
    int a=0;

    void PinocchioCallback(const tocabi_controller::model &msg);
    void jointVelocityEstimate1();

private:
    Eigen::VectorQd ControlVal_;
    Eigen::VectorQd TorqueGrav;
    Eigen::VectorQd TorqueContact;
    Eigen::VectorQd TorqueContact_prev;
    Eigen::VectorQd TorqueContact_notfilter;
    bool filter_init;
};