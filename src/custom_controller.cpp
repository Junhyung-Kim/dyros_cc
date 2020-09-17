#include "custom_controller.h"

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_), wkc_(dc.wkc_)
{
    ControlVal_.setZero();
    wkc_.contactMode == 1.0;
    wkc_.phaseChange = false;
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
    filter_init = true;
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
            wbc_.set_contact(rd_);
            double debug;
            TorqueGrav = wbc_.gravity_compensation_torque(rd_);
            TorqueContact.setZero();
            
            if(wkc_.phaseChange)
            {
                wkc_.rate = DyrosMath::cubic(wkc_.walking_tick,wkc_.double2Single_pre, wkc_.double2Single,1,0,0,0);
                debug =0.1;
                TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, wkc_.rate, wkc_.foot_step(wkc_.current_step_num,6));
            
            }
            else
            {
                if(wkc_.contactMode == 1.0)
                {
                    wkc_.rate = 1;
                    debug =0.2;
                    TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, 1.0, wkc_.foot_step(wkc_.current_step_num,6));
                }
                else
                {
                    wkc_.rate = 0.0;
                    TorqueContact.setZero();
                    debug =0.3;
                }
            }          

            rd_.torque_grav_cc = TorqueGrav + TorqueContact;
            if(filter_init == true)
            {
                TorqueContact_prev = rd_.torque_grav_cc;
                filter_init = false;
            }

            TorqueContact_notfilter = rd_.torque_grav_cc;
            
            for(int i=0; i<MODEL_DOF; i++)
            {
                rd_.torque_grav_cc(i) = 0.012*rd_.torque_grav_cc(i)+(1-0.012)*TorqueContact_prev(i);
            }
            
            TorqueContact_prev = rd_.torque_grav_cc;    

          //  file[0] << debug<<"\t"<<rd_.torque_grav_cc(1)<<"\t"<< TorqueContact_notfilter(1)<<"\t"<< rd_.torque_grav_cc(1)<<"\t"<< filter_init<<"\t"<< wkc_.contactMode<<"\t"<< wkc_.phaseChange<<"\t"<< wkc_.rate<<std::endl;//"\t"<< TorqueContact(2)<<"\t"<<TorqueContact(3)<<"\t"<< TorqueContact(4)<<"\t"<< TorqueContact(5)<<"\t"<<1<<std::endl; 
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
        std::chrono::duration<double> e_s[1];
        std::chrono::high_resolution_clock::time_point t[4];
        if(tc.walking_enable == 1.0)
        {
            t[0] = std::chrono::high_resolution_clock::now();
            if(command_init == true)
            {   
                cycle_count = 0;
                command_init = false;//;cccccc
                wkc_.setRobotStateInitialize();
                wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.walking_pattern2, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, tc.imu_walk, rd_);
                t_begin = std::chrono::high_resolution_clock::now();
                
                for(int i = 12; i<MODEL_DOF; i++)
                {
                    wkc_.desired_init_leg_q(i) = rd_.q_(i);
                }                
            }
           // Eigen::Matrix6d I_temp[LINK_NUMBER-1];
            Eigen::Vector6d temp11, temp12, temp13;
            temp11.setZero();
            temp12.setZero();



 ////////////////////////////////////////////
    double dt;
    dt = 1/1000;
    Eigen::MatrixXd A_t, A_dt, B_t, B_dt, C, I, I_t;
    I.setZero(MODEL_DOF*2,MODEL_DOF*2);
    I.setIdentity();
    I_t.setZero(MODEL_DOF, MODEL_DOF);
    I_t.setIdentity();
    A_t.setZero(MODEL_DOF*2, MODEL_DOF*2);
    A_dt.setZero(MODEL_DOF*2, MODEL_DOF*2);
    B_t.setZero(MODEL_DOF*2, MODEL_DOF);
    B_dt.setZero(MODEL_DOF*2, MODEL_DOF);
    C.setZero(MODEL_DOF, MODEL_DOF*2);

    A_t.topRightCorner(MODEL_DOF, MODEL_DOF);
    A_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = dc_.A_inv.bottomRightCorner(MODEL_DOF, MODEL_DOF) * dc_.tocabi_.Cor_;
    A_t.topRightCorner(MODEL_DOF, MODEL_DOF) = I_t;
    B_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = dc_.A_inv.bottomRightCorner(MODEL_DOF, MODEL_DOF);
    C.bottomLeftCorner(MODEL_DOF, MODEL_DOF) = I_t*dt;
    B_dt = B_t*dt;
    A_dt = I-dt*A_dt;

    
    double L;
    L = 1;

    if(velEst == false)
    {
        q_est = rd_.q_;
        q_dot_est = rd_.q_dot_;
        velEst = true;
    }

    if(velEst = true)
    {
        Eigen::VectorQd q_temp;
        q_temp = q_est;

        q_est = (1+dt)*q_est + L*(rd_.q_ - q_est);  

        q_dot_est = (q_est - q_temp)*1000.0;

        q_dot_est = A_dt.bottomRightCorner(MODEL_DOF, MODEL_DOF)*q_dot_est + B_dt.bottomRightCorner(MODEL_DOF, MODEL_DOF) * dc_.torque_desired - dt*dc_.tocabi_.G.segment<MODEL_DOF>(6);
    }



/////////////////////////////////////////

            wkc_.walkingCompute(rd_);
          /*  Eigen::Vector6d mom_pino;
            mom_pino = dc_.tocabi_.Ag_ * dc_.tocabi_.q_dot_;
            */



       //      file[1] << DyrosMath::rot2Euler((rd_.link_[Left_Foot].Rotm))(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(0) << "\t" << wkc_.RF_trajectory_float.translation()(1) << "\t" << wkc_.LF_trajectory_float.translation()(0) << "\t" << wkc_.RF_float_current.translation()(0) << "\t" << wkc_.RF_float_current.translation()(1) << "\t"<< wkc_.RF_float_current.translation()(2) << "\t"  <<wkc_.Cfsemd(0) << "\t" << wkc_.Cfsemd(1) << "\t"<< wkc_.Cfsemd(2) << "\t" << wkc_.desired_leg_q(7) << "\t" << wkc_.desired_leg_q(8) << "\t" <<wkc_.desired_leg_q(9) << "\t" <<wkc_.desired_leg_q(10) << "\t" <<wkc_.desired_leg_q(11) << std::endl;
   //    file[0] << rd_.com_.vel(0) << "\t" << rd_.com_.vel(1) << "\t"<< rd_.com_.vel(2) << "\t" <<wkc_.MT_cen(0)/95.1703 << "\t"<<wkc_.MT_cen(1)/95.1703 << "\t"<<wkc_.MT_cen(2)/95.1703 << std::endl;
       //      file[0] << dc_.tocabi_.ZMP(0)<<"\t"<< wkc_.aa(0) << "\t" << wkc_.aa(1) << "\t" << wkc_.aa(2) << "\t" << wkc_.aa(3) << "\t" << wkc_.aa(4) << "\t" << 
    //    wkc_.aa(5) << std::endl;

  
            for(int i = 0; i < 12; i++)
            {
                ControlVal_(i) = wkc_.desired_leg_q(i);
            }
            for(int i = 12; i<MODEL_DOF; i++)
            {
                ControlVal_(i) = wkc_.desired_init_leg_q(i);
            }

            ControlVal_(12) = wkc_.q_w(0);
            ControlVal_(13) = wkc_.q_w(1);
            ControlVal_(14) = wkc_.q_w(2);
            ControlVal_(16) = wkc_.q_w(3);
            ControlVal_(26) = wkc_.q_w(4);
  
            t[1] = std::chrono::high_resolution_clock::now();
            e_s[0]= t[0] - t[1];

      file[1] <<     dc_.torque_desired(0) <<"\t" << dc_.torque_desired(1) << "\t" <<  dc_.torque_desired(2) << std::endl;
    //      file[1] <<1 <<"\t"<< q_dot_est(0)<<"\t"<< rd_.q_dot_(0)<<"\t"<< q_dot_est(1)<<"\t"<<rd_.q_dot_(1)<<"\t"<< q_dot_est(2)<<"\t"<< rd_.q_dot_(2)<<"\t"<<std::endl;
         //   std::cout << "time " << e_s[0].count()*1000 << std::endl;
        }
        else if(tc.walking_enable == 3.0)
        {
            if(command_init == true)
            {   
                cycle_count = 0;
                command_init = false;
                wkc_.setRobotStateInitialize();
                wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.walking_pattern2, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, tc.imu_walk, rd_);
                t_begin = std::chrono::high_resolution_clock::now();
            }

            wkc_.walkingCompute(rd_);
            for(int i = 0; i < MODEL_DOF; i++)
            {
                ControlVal_(i) = wkc_.desired_init_leg_q(i);
            }
        }
    }
}

/*
Kp: [3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 
     3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 
     5000.0, 5000.0, 5000.0, 
     500.0, 500.0, 500.0, 500.0, 500.0, 30.0, 20.0, 20.0, 
     500.0, 500.0, 
     500.0, 500.0, 500.0, 500.0, 500.0, 30.0, 20.0, 20.0] 
Kv: [70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 
     70.0, 70.0, 70.0, 70.0, 70.0, 70.0,
     70.0, 70.0, 70.0, 
     20.0, 20.0, 20.0, 20.0, 20.0, 2.0, 3.0, 3.0, 
     20.0, 20.0,
     20.0, 20.0, 20.0, 20.0, 20.0, 2.0, 3.0, 3.0]
*/