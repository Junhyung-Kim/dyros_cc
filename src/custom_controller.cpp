#include "custom_controller.h"

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_), wkc_(dc.wkc_)
{
    ControlVal_.setZero();

    for(int i=0; i<2;i++)
    {
      file[i].open(FILE_NAMES[i].c_str(),std::ios_base::out);
    }
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::taskCommandToCC(TaskCommand tc_)
{
    tc = tc_;
    command_init = true;
}

void CustomController::computeSlow()
{
    if (tc.mode == 10)
    {
        //rd_.control_time_; current time
        //rd_.link_[Right_Foot].Jac : current rightfoot jac
        //rd_.q_dot_ : current q velocity

        //rd_.link_[Right_Foot]

        //ControlVal_=

        wbc_.set_contact(rd_, 1, 1);

        int task_number = 6;
        rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
        rd_.f_star.setZero(task_number);

        rd_.J_task = rd_.link_[Pelvis].Jac;

        rd_.link_[Pelvis].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
        rd_.link_[Pelvis].x_desired(2) = tc.height + tc.ratio * rd_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos(2);
        rd_.link_[Pelvis].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

        rd_.f_star = wbc_.getfstar6d(rd_, Pelvis);

        ControlVal_ = wbc_.task_control_torque_QP2(rd_, rd_.J_task, rd_.f_star);
    }
    else if(tc.mode == 11)
    {
        if(tc.walking_enable == 1.0)
        {
            if(wkc_.contactMode == 1.0)
            {
               rd_.ee_[0].contact = 1.0;
               rd_.ee_[1].contact = 1.0;
            }
            else if(wkc_.contactMode == 2.0)
            {
               rd_.ee_[0].contact = 1.0;
               rd_.ee_[1].contact = 0.0;
            }
            else
            {
                rd_.ee_[0].contact = 0.0;
                rd_.ee_[1].contact = 1.0;
            }

            Vector12d fc_redis;
            double fc_ratio = 0.000;
            fc_redis.setZero();
            VectorQd torquegrav1;
            wbc_.set_contact(rd_);
            
            TorqueGrav = wbc_.gravity_compensation_torque(rd_);
            
 /*           if(wkc_.phaseChange)
            {
                wkc_.rate = DyrosMath::cubic(wkc_.walking_tick,wkc_.double2Single_pre, wkc_.double2Single,1,0,0,0);
                TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, wkc_.rate, wkc_.foot_step(wkc_.current_step_num,6));
                TorqueGrav = TorqueGrav + TorqueContact;
            }
            else
            {
                if(wkc_.contactMode == 1.0)
                {
                    TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, 1.0, wkc_.foot_step(wkc_.current_step_num,6));
                    TorqueGrav = TorqueGrav + TorqueContact;    
                }
                else
                {
                    TorqueGrav = TorqueGrav;
                }
            }        */
            rd_.torque_grav_cc = TorqueGrav;
            file[0] << TorqueGrav(0)<<"\t"<< TorqueGrav(1)<<"\t"<< TorqueGrav(2)<<"\t"<< TorqueGrav(3)<<"\t"<< TorqueGrav(4)<<"\t"<< TorqueGrav(5)<<"\t"<< TorqueContact(0)<<"\t"<< TorqueContact(1)<<"\t"<< TorqueContact(2)<<"\t"<<TorqueContact(3)<<"\t"<< TorqueContact(4)<<"\t"<< TorqueContact(5)<<"\t"<<1<<std::endl; 
        } 
        else if(tc.walking_enable == 3.0)
        {
            rd_.ee_[0].contact = 1.0;
            rd_.ee_[1].contact = 1.0;

            TorqueGrav = wbc_.gravity_compensation_torque(rd_);
            rd_.torque_grav_cc = TorqueGrav;
        }   
            
    }
}

void CustomController::computeFast()
{
    if (tc.mode == 10)
    {
    }
    else if (tc.mode == 11)
    {
    }
}

void CustomController::computePlanner()
{
    if(tc.mode == 11)
    {
        //task controller for mode 11 ....
        int walkingHz = 1000;  
        std::chrono::high_resolution_clock::time_point t_finish;
        
        if(tc.walking_enable == 1.0)
        {
            if(command_init == true)
            {   
                cycle_count = 0;
                command_init = false;
                wkc_.setRobotStateInitialize();
                wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, rd_);
                t_begin = std::chrono::high_resolution_clock::now();
            }
        
            wkc_.walkingCompute(rd_);
    //        file[0] << wkc_.PELV_trajectory_float.translation()(0)<<"\t"<< wkc_.PELV_trajectory_float.translation()(1)<<"\t"<< wkc_.PELV_trajectory_float.translation()(2)<<"\t"<< wkc_.LF_trajectory_float.translation()(0)<<"\t"<< wkc_.LF_trajectory_float.translation()(1)<<"\t"<< wkc_.LF_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_float.translation()(0)<<"\t"<< wkc_.RF_trajectory_float.translation()(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(2)<<"\t"<<std::endl;
        //  file[0] << wkc_.foot_distance(1)<<"\t"<< wkc_.RF_support_current.translation()(0)<<"\t"<<wkc_.PELV_support_current.translation()(1)<<"\t"<<wkc_.PELV_support_current.translation()(2)<<"\t"<< wkc_.PELV_trajectory_float.translation()(0)<<"\t"<< wkc_.PELV_trajectory_float.translation()(1)<<"\t"<< wkc_.PELV_trajectory_float.translation()(2)<<"\t"<< wkc_.LF_trajectory_float.translation()(0)<<"\t"<< wkc_.LF_trajectory_float.translation()(1)<<"\t"<< wkc_.LF_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_float.translation()(0)<<"\t"<< wkc_.RF_trajectory_float.translation()(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_support.translation()(0)<<"\t"<< wkc_.RF_trajectory_support.translation()(1)<<"\t"<< wkc_.RF_trajectory_support.translation()(2)<<std::endl;
        //    file[0]<<wkc_.desired_leg_q(7) <<"\t" <<rd_.q_(0)<<"\t"<<wkc_.desired_leg_q(1) <<"\t"<<rd_.q_(1)<<"\t"<<wkc_.desired_leg_q(2) <<"\t"<<rd_.q_(2)<<"\t"<<wkc_.desired_leg_q(3) <<"\t"<<rd_.q_(3)<<"\t"<<wkc_.desired_leg_q(4) <<"\t"<<rd_.q_(4)<<"\t"<<wkc_.desired_leg_q(5)<<"\t" <<rd_.q_(5)<<std::endl;
        //    file[1]<<wkc_.PELV_support_current.translation()(0)<<"\t"<<wkc_.PELV_support_current.translation()(1)<<"\t"<< wkc_.PELV_support_current.translation()(2)<<"\t"<<wkc_.LF_support_current.translation()(0)<<"\t"<<wkc_.LF_support_current.translation()(1)<<"\t"<<wkc_.LF_support_current.translation()(2)<<"\t"<<wkc_.RF_support_current.translation()(0)<<"\t"<<wkc_.RF_support_current.translation()(1)<<"\t"<<wkc_.RF_support_current.translation()(2)<<"\t"<< wkc_.PELV_trajectory_float.translation()(0)<<"\t"<< wkc_.PELV_trajectory_float.translation()(1)<<"\t"<< wkc_.PELV_trajectory_float.translation()(2)<<"\t"<< wkc_.LF_trajectory_float.translation()(0)<<"\t"<< wkc_.LF_trajectory_float.translation()(1)<<"\t"<< wkc_.LF_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_float.translation()(0)<<"\t"<< wkc_.RF_trajectory_float.translation()(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(2)<<"\t"<<wkc_.foot_step(wkc_.current_step_num,0)<<"\t"<<wkc_.foot_step(wkc_.current_step_num,1)<<std::endl;
   
            for(int i = 0; i < 12; i++)
            {
                ControlVal_(i) = wkc_.desired_leg_q(i);
            }
            for(int i = 12; i<MODEL_DOF; i++)
            {
                ControlVal_(i) = rd_.q_init_(i);
            }
        }
        else if(tc.walking_enable == 3.0)
        {
            if(command_init == true)
            {   
                cycle_count = 0;
                command_init = false;
                wkc_.setRobotStateInitialize();
                wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, rd_);
                t_begin = std::chrono::high_resolution_clock::now();
            }

            wkc_.walkingCompute(rd_);
            for(int i = 0; i < 12; i++)
            {
                ControlVal_(i) = wkc_.desired_leg_q(i);
            }
            for(int i = 12; i<MODEL_DOF; i++)
            {
                ControlVal_(i) = rd_.q_init_(i);
            }
        }
    }
}