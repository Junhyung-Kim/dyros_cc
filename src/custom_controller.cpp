#include "custom_controller.h"
#include <mutex>

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc_.wbc_), wkc_(dc_.wkc_)
{
    ControlVal_.setZero();
    wkc_.contactMode = 1.0;
    walking_tickc = 0;
    torque_prev.setZero();

    dc_.torque_dist.setZero();

    for(int i=0; i<2;i++)
    {
      file[i].open(FILE_NAMES[i].c_str(),std::ios_base::out);
    }
    tocabi_pinocchio = dc_.nh.subscribe("/tocabi/pinocchio", 1, &CustomController::PinocchioCallback, this);
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
            if(walking_tickc>0)
            {    
                if(contactModec == 1.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if(contactModec == 2.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 0.0;
                }
                else
                {
                    rd_.ee_[0].contact = 0.0;
                    rd_.ee_[1].contact = 1.0;
                }
            }
            else
            {
                if(contactModec == 1.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if(contactModec == 2.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 0.0;
                }
                else
                {
                    rd_.ee_[0].contact = 0.0;
                    rd_.ee_[1].contact = 1.0;
                }
            }
            

            Vector12d fc_redis;
            double fc_ratio = 0.000;
            fc_redis.setZero();
            wbc_.set_contact(rd_);
            double debug;
            TorqueGrav = wbc_.gravity_compensation_torque(rd_);
            TorqueContact.setZero();

            if(dc_.mode == "realrobot")
            {
                TorqueGrav(1) = 1.2*TorqueGrav(1);
                TorqueGrav(7) = 1.2*TorqueGrav(7);
            }
            
            if(walking_tickc > 0)
            {
                if(phaseChangec == true && phaseChangec1 == false)
                {  
                    rate = DyrosMath::cubic(walking_tickc,double2Single_prec, double2Singlec,1,0,0,0);
                    TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(current_step_numc,6));
                }
                else if(phaseChangec == false && phaseChangec1 == true)
                {
                    rate = DyrosMath::cubic(walking_tickc,single2Double_prec, single2Doublec,0,1,0,0);
                    if(current_step_numc < total_step_numc -1)
                        TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(current_step_numc ,6));
                    else
                        TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(total_step_numc - 1 ,6));
                }
                else
                {
                    rate = 1.0;
                    TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(current_step_numc,6));                  
                }
            } 

            if(walking_tickc == 0)
            {
                torque_dis.setZero();
                Int_dis.setZero();
                torque_dis_prev.setZero();
            }
            
            Eigen::VectorQd torque_pd;

            torque_pd = dc_.torque_desired - rd_.torque_grav_cc;
            double L__ = 10;

            Int_dis += (dc_.tocabi_.Cor_.transpose()*rd_.q_dot_est1/**rd_.q_dot_virtual_.segment<MODEL_DOF>(6)*/ - dc_.tocabi_.G_ + dc_.torque_desired + torque_dis)/2000;

            torque_dis = L__*(dc_.tocabi_.M_p *rd_.q_dot_est1/* *rd_.q_dot_virtual_.segment<MODEL_DOF>(6)*/-Int_dis);

           torque_dis = 0.9*torque_dis + 0.1*torque_dis_prev;

            Eigen::VectorQd m_gain;
            m_gain.setZero();

            for(int i=0; i<12; i++)
            {
                m_gain(i) = 0.4;
            }
           
            for(int i=12; i<MODEL_DOF; i++)
            {
                m_gain(i) = 0.4;
            }
/*
            mtx_wlk.lock();
            for(int i=0; i<MODEL_DOF; i++)
            {
                dc_.torque_dist(i) = m_gain(i)*torque_dis(i);
            }

             mtx_wlk.unlock();*/

            for(int i = 0; i < 12; i++)
            {
           //     file[1]<<torque_tt(0)<<"\t"<< torque_dis(0) << "\t"<<torque_tt(1)<<"\t" << torque_dis(1)<<"\t"<<torque_tt(2)<< "\t" << torque_dis(2)<< "\t"<<torque_tt(3)<<"\t" << torque_dis(3)<< "\t"<<torque_tt(4)<<"\t" << torque_dis(4)<<"\t"<<torque_tt(5)<< "\t" << torque_dis(5)<<std::endl;
         //       file[1] << dc_.torque_desired(0) <<"\t" << dc_.tt(0)<<"\t"<<torque_tt(6)<<"\t"<<dc_.torqueElmo(6)<<"\t" << torque_dis(1) <<"\t" << dc_.torque_desired(1)<<"\t"<<torque_tt(7)<<"\t"<<dc_.torqueElmo(7)<<"\t" << torque_dis(2) <<"\t" << dc_.torque_desired(2)<<"\t"<<torque_pd(8)<<"\t"<<dc_.torqueElmo(8)<<"\t" << torque_dis(3) <<"\t" << dc_.torque_desired(3)<<"\t"<<torque_pd(9)<<"\t"<<dc_.torqueElmo(3)<<"\t" << torque_dis(4) <<"\t" << dc_.torque_desired(4)<<"\t" << torque_dis(5) <<"\t" << dc_.torque_desired(5)<<std::endl;
   //      file[1] << wkc_.PELV_trajectory_float.translation()(0) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(0)<<"\t"<< wkc_.PELV_trajectory_float.translation()(1) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(1)<<"\t"<< wkc_.PELV_trajectory_float.translation()(2) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_float.translation()(0) <<"\t"<<wkc_.RFD_trajectory_float.translation()(0)<<"\t"<< wkc_.RF_trajectory_float.translation()(1) <<"\t"<<wkc_.RFD_trajectory_float.translation()(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(2) <<"\t"<<wkc_.RFD_trajectory_float.translation()(2)<<"\t"<< wkc_.LF_trajectory_float.translation()(0) <<"\t"<<wkc_.LFD_trajectory_float.translation()(0)<<"\t"<< wkc_.LF_trajectory_float.translation()(1) <<"\t"<<wkc_.LFD_trajectory_float.translation()(1)<<"\t"<< wkc_.LF_trajectory_float.translation()(2) <<"\t"<<wkc_.LFD_trajectory_float.translation()(2)<<"\t"<<wkc_.contactMode<<"\t"<<contactModec<<"\t"<<wkc_.phaseChange<<"\t"<<phaseChangec<<"\t"<<rate<<"\t"<<wkc_.foot_step(wkc_.current_step_num,6)<<"\t"<<foot_stepc(current_step_numc,6)<<"\t"<<wkc_.current_step_num<<"\t"<<current_step_numc<<"\t"<<TorqueContact(0)<<std::endl;   
              //  file[1] <<debug<<"\t"<<wkc_.walking_tick<<"\t"<<TorqueGrav(3) <<"\t"<<TorqueContact(3)<<"\t"<<rd_.torque_grav_cc(3)<<"\t"<<rd_.torque_grav_cc(1)<<"\t"<<rd_.torque_grav_cc(2)<<"\t"<<rd_.torque_grav_cc(3)<<"\t"<<rd_.torque_grav_cc(4)<<"\t"<<rd_.torque_grav_cc(5)<<"\t"<<phaseChangec1 << "\t" <<phaseChangec <<"\t"<<contactModec<<"\t"<< current_step_numc <<"\t"<<wkc_.LF_trajectory_float.translation()(2)<<"\t"<<wkc_.RF_trajectory_float.translation()(2)<<"\t"<<rate<<"\t"<<single2Double_prec<<"\t"<<single2Doublec<<"\t"<<wkc_.double2Single_pre<<"\t"<<wkc_.double2Single<<"\t"<<S2DChangec<<std::endl;
                //std::endl;//TorqueContact(0)<<"\t"<<wkc_.final_posx(0)<<"\t"<<wkc_.PELV_trajectory_float.translation()(0)<<"\t"<<rd_.link_[Pelvis].xipos(0)<<"\t"<<rd_.ZMP(0)<<std::endl;
  //          file[1] << walking_tickc <<"\t"<<rd_.torque_grav_cc(0)<<"\t" << rd_.torque_grav_cc(1)<<"\t" << rd_.torque_grav_cc(2)<<"\t" << rd_.torque_grav_cc(3)<<"\t" << rd_.torque_grav_cc(4)<<"\t" << rd_.torque_grav_cc(5)<<"\t"<<phaseChangec <<"\t"<<phaseChangec1<<"\t"<<single2Double_prec<<"\t"<<single2Doublec<<"\t"<<double2Single_prec<<"\t"<<double2Singlec<<"\t"<<rate<<"\t"<<contactModec<<"\t"<<wkc_.RF_trajectory_float.translation()(2)<<"\t"<<wkc_.LF_trajectory_float.translation()(2)<<std::endl;
//  file[1] <<TorqueContact(0)<<"\t"<<TorqueContact(1)<<"\t"<<TorqueContact(2)<<"\t"<<TorqueContact(3)<<"\t"<<TorqueContact(4)<<"\t"<<TorqueContact(5)<<std::endl;
            }

            rd_.torque_grav_cc = TorqueGrav + TorqueContact;


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
        if(tc.walking_enable == 1.0)
        {                      
           
        }
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
                command_init = false;
                wkc_.setRobotStateInitialize(rd_);
                wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.walking_pattern2, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, tc.imu_walk, tc.mom, tc.vibration_control, rd_);
                t_begin = std::chrono::high_resolution_clock::now();

                for (int i = 12; i < MODEL_DOF; i++)
                {
                    wkc_.desired_init_leg_q(i) = rd_.q_(i);
                }
            }

            Eigen::Vector6d temp11, temp12, temp13;
            temp11.setZero();
            temp12.setZero();

            wkc_.walkingCompute(rd_);
  
            mtx_wlk.lock();
            current_step_numc = wkc_.current_step_num;
            total_step_numc = wkc_.total_step_num;
            walking_tickc = wkc_.walking_tick;
            foot_stepc = wkc_.foot_step;
            contactModec = wkc_.contactMode;
            phaseChangec = wkc_.phaseChange;
            phaseChangec1 = wkc_.phaseChange1;
            double2Single_prec = wkc_.double2Single_pre;
            double2Singlec = wkc_.double2Single;
            single2Double_prec = wkc_.single2Double_pre;
            single2Doublec = wkc_.single2Double;

            mtx_wlk.unlock();

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

           rd_.q_dot_desired_.setZero();


        /*    Eigen::Vector3d cmp;
            if(contactModec == 1)
            {
                cmp(1) = 0.0;
            }
            else if(contactModec == 2)
            { // LeFt support
                cmp(1) = rd_.com_.pos(1) -(rd_.ContactForce_FT(1))/(rd_.ContactForce_FT(2))*wkc_.zc;
            }
            else
            {
                cmp(1) = rd_.com_.pos(1) -(rd_.ContactForce_FT(7))/(rd_.ContactForce_FT(8))*wkc_.zc;
            }*/
            Eigen::VectorQd torque_tt;

            torque_tt = (- dc_.torque_desired + dc_.torqueElmo);

            Eigen::Vector3d desired_ankle_torque , pr, pl, ZMP_ft, com_moment, zmp2com;
            Eigen::Vector2d x_temp, L_temp, cmp;
            double L__, L_temp1, A__;    

            pr(2) = 0.0;
            pl(2) = 0.0;
            ZMP_ft(2) = 0.0;

            for(int i = 0; i<2;i++)
            {
                pr(i) = rd_.link_[Right_Foot].xipos(i);
                pl(i) = rd_.link_[Left_Foot].xipos(i);
                ZMP_ft(i) = rd_.ZMP_ft(i); 
            }

          //  ZMP_ft(0) = ZMP_ft(0) - 0.03;

            zmp2com(0) = rd_.com_.pos(0) - ZMP_ft(0);
            zmp2com(0) = rd_.com_.pos(1) - ZMP_ft(1);
            zmp2com(0) = rd_.com_.pos(2);


            if(wkc_.contactMode == 1.0)
            {
                cmp(0) = rd_.com_.pos(0) - (rd_.ContactForce(0)+rd_.ContactForce(6))/(rd_.ContactForce(2)+rd_.ContactForce(8))*rd_.com_.pos(2);
                cmp(1) = rd_.com_.pos(1) - (rd_.ContactForce(1)+rd_.ContactForce(7))/(rd_.ContactForce(2)+rd_.ContactForce(8))*rd_.com_.pos(2);
            }
            else if(wkc_.contactMode == 2.0)
            {//Leftsupport
                cmp(0) = rd_.com_.pos(0) - rd_.ContactForce(0)/rd_.ContactForce(2)*rd_.com_.pos(2);
                cmp(1) = rd_.com_.pos(1) - rd_.ContactForce(1)/rd_.ContactForce(2)*rd_.com_.pos(2);
            }
            else
            {
                cmp(0) = rd_.com_.pos(0) - rd_.ContactForce(6)/rd_.ContactForce(8)*rd_.com_.pos(2);
                cmp(1) = rd_.com_.pos(1) - rd_.ContactForce(7)/rd_.ContactForce(8)*rd_.com_.pos(2);               
            }
            



            A__ = (pr(1)-pl(1))/(pr(0)-pl(0));
            L_temp1 = sqrt((pr(1)-pl(1))*(pr(1)-pl(1))+(pr(0)-pl(0))*(pr(0)-pl(0)));

            if(wkc_.contactMode == 1)
            {
                L_temp1 = sqrt((pr(1)-pl(1))*(pr(1)-pl(1))+(pr(0)-pl(0))*(pr(0)-pl(0)));

                x_temp(0) = (A__*pl(0)+ZMP_ft(0)/A__+ZMP_ft(1)-pl(1))/(A__+1/A__);
                x_temp(1) = A__*(x_temp(0)-pl(0))+pl(1);

                if(x_temp(1) > pl(1))
                {
                    x_temp(0) = pl(0);
                    x_temp(1) = pl(1);
                }

                if(x_temp(1) < pr(1))
                {
                    x_temp(0) = pr(0);
                    x_temp(1) = pr(1);
                }

                L__ = sqrt((x_temp(0) - pl(0))*(x_temp(0) - pl(0))+(x_temp(1) - pl(1))*(x_temp(1) - pl(1)))/L_temp1;
//                L__= sqrt(L_temp(0)*L_temp(0)+L_temp(1)*L_temp(1))/L_temp1;
            }
            else if(wkc_.contactMode == 2)
            {
                ///LEFT supports
                L_temp(0) = 0.0;
                L_temp(1) = 0.0;
                x_temp(0) = pl(0);
                x_temp(1) = pl(1);
                L__ = 0.0;
            }
            else
            {

                L_temp(0) = 1.0;
                L_temp(1) = 1.0;
                x_temp(0) = pr(0);
                x_temp(1) = pr(1);
                L__ = 1.0;
            }

            Eigen::Vector3d tt__;
            tt__(2) = 0;
            tt__.segment<2>(0) = x_temp;

            desired_ankle_torque = -DyrosMath::skew((pr - tt__))*rd_.ContactForce.segment<3>(6) - DyrosMath::skew((pl - tt__))*rd_.ContactForce.segment<3>(0);

            Eigen::Vector2d RT, LT;

            dc_.torque_dist.setZero();

            mtx_wlk.lock();

            RT = L__ *desired_ankle_torque.segment<2>(0);
            LT = (1-L__) *desired_ankle_torque.segment<2>(0);
/*
            dc_.torque_dist(5) = 0.6*LT(0);
            dc_.torque_dist(4) = 0.6*LT(1);


            dc_.torque_dist(11) = 0.6*RT(0);
            dc_.torque_dist(10) = 0.6*RT(1);
*/

            dc_.torque_dist(5) = 1.0*LT(0);
            dc_.torque_dist(4) = 1.0*LT(1);


            dc_.torque_dist(11) = 1.0*RT(0);
            dc_.torque_dist(10) = 1.0*RT(1);

/*            for(int i=0; i<MODEL_DOF; i++)
            {
                dc_.torque_dist(i) = m_gain(i)*torque_dis(i);
            }
*/
             mtx_wlk.unlock();

            if(walking_tickc >0)
            {
       //         file[1] << torque_tt(0) << "\t"<< torque_tt(1) << "\t"<< torque_tt(2) << "\t"<< torque_tt(3) << "\t"<< torque_tt(4) << "\t"<< torque_tt(5) << "\t"<<torque_dis(0)<< "\t"<<torque_dis(1)<< "\t"<<torque_dis(2)<< "\t"<<torque_dis(3)<< "\t"<<torque_dis(4)<< "\t"<<torque_dis(5)<<std::endl;
//                file[1] << torque_dis(0) <<"\t" << dc_.torque_desired(0)<<"\t"<<torque_pd(6)<<"\t"<<dc_.torqueElmo(6)<<"\t" << torque_dis(1) <<"\t" << dc_.torque_desired(1)<<"\t"<<torque_pd(7)<<"\t"<<dc_.torqueElmo(7)<<"\t" << torque_dis(2) <<"\t" << dc_.torque_desired(2)<<"\t"<<torque_pd(8)<<"\t"<<dc_.torqueElmo(8)<<"\t" << torque_dis(3) <<"\t" << dc_.torque_desired(3)<<"\t"<<torque_pd(9)<<"\t"<<dc_.torqueElmo(3)<<"\t" << torque_dis(4) <<"\t" << dc_.torque_desired(4)<<"\t" << torque_dis(5) <<"\t" << dc_.torque_desired(5)<<std::endl;
      //   file[1] << wkc_.PELV_trajectory_float.translation()(0) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(0)<<"\t"<< wkc_.PELV_trajectory_float.translation()(1) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(1)<<"\t"<< wkc_.PELV_trajectory_float.translation()(2) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_float.translation()(0) <<"\t"<<wkc_.RFD_trajectory_float.translation()(0)<<"\t"<< wkc_.RF_trajectory_float.translation()(1) <<"\t"<<wkc_.RFD_trajectory_float.translation()(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(2) <<"\t"<<wkc_.RFD_trajectory_float.translation()(2)<<"\t"<< wkc_.LF_trajectory_float.translation()(0) <<"\t"<<wkc_.LFD_trajectory_float.translation()(0)<<"\t"<< wkc_.LF_trajectory_float.translation()(1) <<"\t"<<wkc_.LFD_trajectory_float.translation()(1)<<"\t"<< wkc_.LF_trajectory_float.translation()(2) <<"\t"<<wkc_.LFD_trajectory_float.translation()(2)<<"\t"<<wkc_.contactMode<<"\t"<<contactModec<<"\t"<<wkc_.phaseChange<<"\t"<<phaseChangec<<"\t"<<rate<<"\t"<<wkc_.foot_step(wkc_.current_step_num,6)<<"\t"<<foot_stepc(current_step_numc,6)<<"\t"<<wkc_.current_step_num<<"\t"<<current_step_numc<<"\t"<<TorqueContact(0)<<std::endl;   
              //  file[1] <<debug<<"\t"<<wkc_.walking_tick<<"\t"<<TorqueGrav(3) <<"\t"<<TorqueContact(3)<<"\t"<<rd_.torque_grav_cc(3)<<"\t"<<rd_.torque_grav_cc(1)<<"\t"<<rd_.torque_grav_cc(2)<<"\t"<<rd_.torque_grav_cc(3)<<"\t"<<rd_.torque_grav_cc(4)<<"\t"<<rd_.torque_grav_cc(5)<<"\t"<<phaseChangec1 << "\t" <<phaseChangec <<"\t"<<contactModec<<"\t"<< current_step_numc <<"\t"<<wkc_.LF_trajectory_float.translation()(2)<<"\t"<<wkc_.RF_trajectory_float.translation()(2)<<"\t"<<rate<<"\t"<<single2Double_prec<<"\t"<<single2Doublec<<"\t"<<wkc_.double2Single_pre<<"\t"<<wkc_.double2Single<<"\t"<<S2DChangec<<std::endl;
                //std::endl;//TorqueContact(0)<<"\t"<<wkc_.final_posx(0)<<"\t"<<wkc_.PELV_trajectory_float.translation()(0)<<"\t"<<rd_.link_[Pelvis].xipos(0)<<"\t"<<rd_.ZMP(0)<<std::endl;
  //          file[1] << walking_tickc <<"\t"<<rd_.torque_grav_cc(0)<<"\t" << rd_.torque_grav_cc(1)<<"\t" << rd_.torque_grav_cc(2)<<"\t" << rd_.torque_grav_cc(3)<<"\t" << rd_.torque_grav_cc(4)<<"\t" << rd_.torque_grav_cc(5)<<"\t"<<phaseChangec <<"\t"<<phaseChangec1<<"\t"<<single2Double_prec<<"\t"<<single2Doublec<<"\t"<<double2Single_prec<<"\t"<<double2Singlec<<"\t"<<rate<<"\t"<<contactModec<<"\t"<<wkc_.RF_trajectory_float.translation()(2)<<"\t"<<wkc_.LF_trajectory_float.translation()(2)<<std::endl;
//  file[1] <<TorqueContact(0)<<"\t"<<TorqueContact(1)<<"\t"<<TorqueContact(2)<<"\t"<<TorqueContact(3)<<"\t"<<TorqueContact(4)<<"\t"<<TorqueContact(5)<<std::endl;
            }
// file[1]  <<wkc_.capturePoint_ox(wkc_.current_step_num) <<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t"<<wkc_.capturePoint_refx(wkc_.walking_tick)<<"\t"<<wkc_.capturePoint_refy(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refx(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refy(wkc_.walking_tick)<<"\t"<<pr(0)<<"\t"<<pr(1)<<"\t"<<pl(0)<<"\t"<<pl(1)<<std::endl;
        
       // file[1] <<x_temp(0)<<"\t"<<x_temp(1)<<"\t"<< LT(0) <<"\t" << LT(1) <<"\t"<< RT(0) <<"\t" << RT(1) <<"\t"<<ZMP_ft(0) << "\t"<<ZMP_ft(1)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t"<<wkc_.capturePoint_refx(wkc_.walking_tick)<<"\t"<<wkc_.capturePoint_refy(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refx(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refy(wkc_.walking_tick)<<"\t"<<wkc_.zmp_dy(wkc_.current_step_num)<<"\t"<<pr(0)<<"\t"<<pr(1)<<"\t"<<pl(0)<<"\t"<<pl(1)<<std::endl;
         //   file[1] << x_temp(0) <<"\t" << x_temp(1) << "\t"<<ZMP_ft(0) << "\t"<<ZMP_ft(1)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t"<<rd_.com_.pos(0)<<"\t"<<rd_.com_.pos(1)<<"\t"<<wkc_.zmp_refx(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refy(wkc_.walking_tick)<<"\t" << rd_.link_[Pelvis].xipos(0)<<"\t" << rd_.link_[Pelvis].xipos(1) <<"\t" << rd_.link_[COM_id].xipos(0)<<"\t" << rd_.link_[COM_id].xipos(1) <<std::endl;

         //////////////1209///
  //     file[1]<<ZMP_ft(0) << "\t"<<ZMP_ft(1)<<"\t"<<wkc_.zmp_refx(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refy(wkc_.walking_tick)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t" << rd_.link_[Pelvis].xipos(0)<<"\t" << rd_.link_[Pelvis].xipos(1) <<"\t"<<rd_.com_.pos(0)-0.0287<<"\t"<<rd_.com_.pos(1)<<"\t"<<rd_.com_.pos(2)<<"\t"<<wkc_.PELV_trajectory_float.translation()(0)<<"\t"<<wkc_.PELV_trajectory_float.translation()(1)<<std::endl;


         file[1] <<ZMP_ft(0) <<"\t"<<rd_.com_.pos(0) <<"\t"<<rd_.com_.angular_moment(1)<<"\t"<<cmp(0)<<"\t" << cmp(1) <<std::endl;//"\t" << cmp_t(0) << "\t" << cmp_t(1) << std::endl;

  //     file[1] <<wkc_.PELV_trajectory_float.translation()(0)<<"\t"<<wkc_.PELV_trajectory_float.translation()(1)<<"\t"<< wkc_.com_refy(wkc_.walking_tick) << "\t" << rd_.com_.pos(1)<<"\t" << rd_.link_[Pelvis].xipos(1) <<"\t" << wkc_.xy_vib_est(0)<<std::endl;
           //   "\t" << rd_.link_[Left_Foot].xipos(0) << "\t" << rd_.link_[Left_Foot].xipos(1) << "\t" << rd_.link_[Left_Foot].xipos(2)<< "\t" << rd_.link_[Right_Foot].xipos(0) << "\t" << rd_.link_[Right_Foot].xipos(1)<< "\t" << rd_.link_[Right_Foot].xipos(2)
        //file[1]<<dc_.torque_dist(4)<<"\t"<<dc_.torque_dist(5)<<"\t"<<dc_.torque_dist(10)<<"\t"<<dc_.torque_dist(11)<<"\t"<< L__<<"\t"<<ZMP_ft(0)<<"\t"<<ZMP_ft(1)<<"\t"<< L__ <<"\t" <<desired_ankle_torque(0) <<"\t" << desired_ankle_torque(1) <<"\t" << desired_ankle_torque(2) <<std::endl;

            t[1] = std::chrono::high_resolution_clock::now();
            e_s[0]= t[0] - t[1];
        //    file[1] << wkc_.desired_leg_q(0)<<"\t" << rd_.q_(0)<<"\t" << wkc_.desired_leg_q(1)<<"\t" << rd_.q_(1)<<"\t" << wkc_.desired_leg_q(2)<<"\t" << rd_.q_(2)<<"\t" << wkc_.desired_leg_q(3)<<"\t" << rd_.q_(3)<<"\t" << wkc_.desired_leg_q(4)<<"\t" << rd_.q_(4)<<"\t" << wkc_.desired_leg_q(5)<<"\t" << rd_.q_(5)<<"\t"<<rd_.q_(12)<<"\t"<<rd_.q_(13)<<"\t"<<rd_.q_(14)<<"\t"<<rd_.q_(15)<<"\t"<<rd_.q_(16)<<"\t"<<rd_.q_(17)<<"\t"<<rd_.q_(18)<<"\t"<<rd_.q_(19)<<"\t"<<rd_.q_(20)<<"\t"<<rd_.q_(21)<<"\t"<<rd_.q_(22)<<"\t"<<rd_.q_(23)<<"\t"<<rd_.q_(24)<<"\t"<<rd_.q_(25)<<"\t"<<rd_.q_(26)<<"\t"<<rd_.q_(27)<<"\t"<<rd_.q_(28)<<"\t"<<rd_.q_(29)<<"\t"<<rd_.q_(30)<<"\t"<<rd_.q_(31)<<"\t"<<rd_.q_(32)<<"\t"<<std::endl;
       //     file[1] << rd_.q_(12)<<"\t"<< rd_.q_(13)<<"\t"<< rd_.q_(14)<<"\t"<< rd_.q_(15)<<"\t"<< rd_.q_(16)<<"\t"<< rd_.q_(17)<<"\t"<< rd_.q_(18)<<"\t"<< rd_.q_(19)<<"\t"<< rd_.q_(20)<<"\t"<< rd_.q_(21)<<std::endl;
      //file[1] <<wkc_.RF_trajectory_float.translation()(2) <<"\t" << wkc_.LF_trajectory_float.translation()(2)<<"\t" << wkc_.phaseChange1 <<"\t"<<wkc_.phaseChange<< "\t"<<std::endl;//<< wkc_.foot_step(current_step_numc + 1,6) <<"\t"<< wkc_.total_step_num <<std::endl;
       // file[1] << wkc_.PELV_trajectory_float.translation()(0) << "\t" << wkc_.PELV_trajectory_float.translation()(1) << "\t" <<wkc_.RF_trajectory_float.translation()(0) << "\t" << wkc_.RF_trajectory_float.translation()(1) << "\t" << wkc_.LF_trajectory_float.translation()(0) << "\t" << wkc_.LF_trajectory_float.translation()(1) << "\t" <<wkc_.com_refx(wkc_.walking_tick) << "\t" << wkc_.com_refy(wkc_.walking_tick) << "\t" << rd_.link_[Pelvis].xipos(0) << "\t" << rd_.link_[Pelvis].xipos(1)<< "\t" << rd_.link_[Pelvis].xipos(2)  << "\t" << rd_.link_[Left_Foot].xipos(0) << "\t" << rd_.link_[Left_Foot].xipos(1) << "\t" << rd_.link_[Left_Foot].xipos(2)<< "\t" << rd_.link_[Right_Foot].xipos(0) << "\t" << rd_.link_[Right_Foot].xipos(1)<< "\t" << rd_.link_[Right_Foot].xipos(2)  <<  "\t" << wkc_.final_posx(0) <<"\t"<<wkc_.xx_vib_est(0)<<"\t"<<wkc_.xx_vib_est(1)<<"\t"<<rd_.ZMP(0)<< "\t"<<rd_.ZMP_ft(0)<<"\t"<<rd_.link_[Pelvis].xipos(0)<<"\t"<<rd_.link_[Pelvis].v(0)<<std::endl;
       //     file[1] << rd_.link_[Pelvis].xipos(1) <<"\t" << rd_.ZMP(1) << std::endl;
        //    file[1] <<(wkc_.RF_fisrt_init).translation()(0)  << "\t" << wkc_.capturePoint_ox(2) << "\t" << wkc_.capturePoint_ox(3)<< "\t" << wkc_.foot_step(1,0) <<"\t"<<wkc_.foot_step(0,0) <<"\t"<< wkc_.PELV_trajectory_float.translation()(0)<<"\t" << wkc_.PELV_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_float.translation()(0)<<"\t" << wkc_.RF_trajectory_float.translation()(2)<<"\t"<< rd_.q_dot_est(2)<<"\t" << rd_.q_dot_(2)<<"\t"<<rd_.q_dot_est(3)<<"\t" << rd_.q_dot_(3)<<"\t"<< wkc_.q_dm(4)<<"\t" <<std::endl;  //"\t"<< rd_.q_dot_(2)<<"\t"<<std::endl;
     //   file[1] <<wkc_.com_refdx(wkc_.walking_tick)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<rd_.com_.pos(0)<<"\t"<<wkc_.H_leg(1)<<"\t"<<ControlVal_(13)<<"\t"<<wkc_.q_dm(1)<<"\t"<<DyrosMath::rot2Euler(rd_.link_[0].Rotm)(0)<<"\t" <<rd_.link_[0].xpos(2)-rd_.link_[4].xpos(2)<<"\t"<<rd_.link_[5].xpos(0)<< "\t"<<wkc_.PELV_trajectory_float.translation()(0) << "\t" << wkc_.PELV_trajectory_float.translation()(1) << "\t"<< wkc_.RF_trajectory_float.translation()(0) << "\t" << wkc_.RF_trajectory_float.translation()(1) << "\t" << wkc_.com_refx(wkc_.walking_tick) << "\t" << rd_.link_[Pelvis].xipos(0) << "\t" << rd_.link_[Pelvis].xipos(1)<< "\t" << rd_.link_[Pelvis].xipos(2)  << "\t" << rd_.link_[Left_Foot].xipos(0) << "\t" << rd_.link_[Left_Foot].xipos(1) << "\t" << rd_.link_[Left_Foot].xipos(2)<< "\t" << rd_.link_[Right_Foot].xipos(0) << "\t" << rd_.link_[Right_Foot].xipos(1)<< "\t" << rd_.link_[Right_Foot].xipos(2)  <<  "\t" << ControlVal_(4) <<"\t"<<rd_.q_(4)<< "\t" << ControlVal_(5) <<"\t" << rd_.q_(5)<< std::endl;
    // file[1] << wkc_.PELV_trajectory_float.translation()(0) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(0)<<"\t"<< wkc_.PELV_trajectory_float.translation()(1) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(1)<<"\t"<< wkc_.PELV_trajectory_float.translation()(2) <<"\t"<<wkc_.PELVD_trajectory_float.translation()(2)<<"\t"<< wkc_.RF_trajectory_float.translation()(0) <<"\t"<<wkc_.RFD_trajectory_float.translation()(0)<<"\t"<< wkc_.RF_trajectory_float.translation()(1) <<"\t"<<wkc_.RFD_trajectory_float.translation()(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(2) <<"\t"<<wkc_.RFD_trajectory_float.translation()(2)<<"\t"<< wkc_.LF_trajectory_float.translation()(0) <<"\t"<<wkc_.LFD_trajectory_float.translation()(0)<<"\t"<< wkc_.LF_trajectory_float.translation()(1) <<"\t"<<wkc_.LFD_trajectory_float.translation()(1)<<"\t"<< wkc_.LF_trajectory_float.translation()(2) <<"\t"<<wkc_.LFD_trajectory_float.translation()(2)<<"\t"<<wkc_.contactMode<<"\t"<<wkc_.phaseChange<<"\t"<<wkc_.rate<<"\t"<<wkc_.foot_step(wkc_.current_step_num,6)<<"\t"<<wkc_.current_step_num<<std::endl;   
       // file[1] <<rd_.ee_[0].contact <<"\t"<< rd_.ee_[1].contact<<"\t"<< rd_.ContactForce(0) <<"\t"<< dc_.torque_desired(0) <<"\t"<< dc_.torque_desired(1) <<"\t"<< dc_.torque_desired(2) <<"\t"<< dc_.torque_desired(3) <<"\t"<< dc_.torque_desired(4) <<"\t"<<rd_.ContactForce(5)<<std::endl;
    //  //  file[1] << wkc_.PELV_trajectory_float.translation()(0) << "\t" << wkc_.PELV_trajectory_float.translation()(1) << "\t" <<wkc_.RF_trajectory_float.translation()(0) << "\t" << wkc_.RF_trajectory_float.translation()(1) << "\t" << wkc_.LF_trajectory_float.translation()(0) << "\t" << wkc_.LF_trajectory_float.translation()(1) << "\t" <<wkc_.com_refx(wkc_.walking_tick) << "\t" << wkc_.com_refy(wkc_.walking_tick) << "\t" << rd_.link_[Pelvis].xipos(0) << "\t" << rd_.link_[Pelvis].xipos(1)<< "\t" << rd_.link_[Pelvis].xipos(2)  << "\t" << rd_.link_[Left_Foot].xipos(0) << "\t" << rd_.link_[Left_Foot].xipos(1) << "\t" << rd_.link_[Left_Foot].xipos(2)<< "\t" << rd_.link_[Right_Foot].xipos(0) << "\t" << rd_.link_[Right_Foot].xipos(1)<< "\t" << rd_.link_[Right_Foot].xipos(2)  <<  "\t" << wkc_.final_posx(0) <<"\t"<<wkc_.xx_vib_est(0)<<"\t"<<wkc_.xx_vib_est(1)<<"\t"<<rd_.ZMP(0)<< "\t" <<rd_.ZMP(1)<<"\t"<<rd_.ZMP_ft(0)<<"\t"<<rd_.ZMP_ft(1)<<"\t"<<rd_.link_[Pelvis].xipos(0)<<"\t"<<rd_.link_[Pelvis].v(0)<<"\t"<<wkc_.final_posx(0)<<"\t"<<cmp(1)<<"\t"<<rd_.com_.angular_momentum(0)<<"\t"<<rd_.com_.angular_momentum(1)<<"\t"<<rd_.com_.angular_momentum(2)<<std::endl;
        }
        else if(tc.walking_enable == 3.0)
        {
            if(command_init == true)
            {   
                cycle_count = 0;
                command_init = false;
                wkc_.setRobotStateInitialize(rd_);
                wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.walking_pattern2, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, tc.imu_walk, tc.mom, tc.vibration_control, rd_);
                t_begin = std::chrono::high_resolution_clock::now();
            }

            wkc_.walkingCompute(rd_);
            
            rd_.q_dot_desired_.setZero();
            
            for(int i = 0; i < MODEL_DOF; i++)
            {
                ControlVal_(i) = wkc_.desired_init_leg_q(i);
            }
        }
    }
}

void CustomController::PinocchioCallback(const tocabi_controller::model &msg)
{
    for (int i = 0; i<6; i++)
    {
        for(int j=0; j<MODEL_DOF; j++)
        {   
            dc_.tocabi_.Ag_(i,j) = msg.CMM[33*i+j];
        }
    }

    for (int i = 0; i<MODEL_DOF; i++)
    {
        for(int j=0; j<MODEL_DOF; j++)
        {   
            dc_.tocabi_.Cor_(i,j) = msg.COR[33*i+j];
        }
    }

    for (int i = 0; i<MODEL_DOF; i++)
    {
        for(int j=0; j<MODEL_DOF; j++)
        {   
            dc_.tocabi_.M_p(i,j) = msg.M[33*i+j];
        }
    }

    for (int i = 0; i<MODEL_DOF; i++)
    {
        dc_.tocabi_.G_(i) = msg.g[i];
    }

    dc_.tocabi_.M_p.triangularView<Eigen::StrictlyLower>() = dc_.tocabi_.M_p.transpose().triangularView<Eigen::StrictlyLower>();
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
     Kp: [7500.0, 8000.0, 7500.0, 7500.0, 5500.0, 5500.0, 
     7500.0, 8000.0, 7500.0, 7500.0, 5500.0, 5500.0, 
     8500.0, 8500.0, 8500.0, 
     6000.0, 6000.0, 6000.0, 1000.0, 1000.0, 1000.0, 10.0, 10.0, 
     300.0, 300.0, 
     6000.0, 6000.0, 6000.0, 1000.0, 1000.0, 1000.0, 10.0, 10.0] 
Kv: [35.0, 15.0, 15.0, 15.0, 25.0, 25.0, 
     35.0, 15.0, 15.0, 15.0, 25.0, 25.0,
     80.0, 80.0, 80.0, 
     45.0, 45.0, 45.0, 45.0, 45.0, 5.0, 1.0, 1.0, 
     10.0, 10.0,
     45.0, 45.0, 45.0, 45.0, 45.0, 5.0, 1.0, 1.0]

*/
