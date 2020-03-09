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
    
    DataContainer &dc_;
    RobotData &rd_;
    WholebodyController &wbc_;
    Walking_controller &wkc_;
    TaskCommand tc;

    bool command_init = false;
    std::chrono::high_resolution_clock::time_point t_begin;
    int cycle_count;

private:
    Eigen::VectorQd ControlVal_;
};
