#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include "math_type_define.h"

class CustomController
{
public:
    CustomController(DataContainer &dc,RobotData &rd);
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


private:
    Eigen::VectorQd ControlVal_;
    Eigen::VectorQd TorqueGrav;
    Eigen::VectorQd TorqueContact;
    Eigen::VectorQd TorqueContact_prev;
    Eigen::VectorQd TorqueContact_notfilter;
    bool filter_init;
};
