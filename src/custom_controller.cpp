#include "custom_controller.h"
#include <mutex>

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc_.wbc_), wkc_(dc_.wkc_)
{
    ControlVal_.setZero();
    wkc_.contactMode = 1.0;
    walking_tickc = 0;
    torque_prev.setZero();

    dc_.torque_dist.setZero();

    for (int i = 0; i < 2; i++)
    {
        file[i].open(FILE_NAMES[i].c_str(), std::ios_base::out);
    }
    tocabi_pinocchio = dc_.nh.subscribe("/tocabi/pinocchio", 1, &CustomController::PinocchioCallback, this);
    joint_pin_pub = dc.nh.advertise<sensor_msgs::JointState>("/tocabi/pinocchio/jointstates", 100);
    dist_pub = dc.nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100);

    joint_state_msg.position.resize(MODEL_DOF + 7);
    joint_state_msg.velocity.resize(MODEL_DOF + 6);
    joint_state_msg.effort.resize(MODEL_DOF + 6);
/*
        if((shmid = shmget((key_t)KEY_NUM, MEM_SIZE, IPC_CREAT| IPC_EXCL | 0666)) == -1) 
        {
            printf("There was shared memory.");
            ROS_INFO("There was shared memory.");

            shmid = shmget((key_t)KEY_NUM, MEM_SIZE, IPC_CREAT| 0666);
            
            if(shmid == -1)
            {
                perror("Shared memory create fail");
                ROS_INFO("Shared memory create fail");
                //return 1;
            }
            else
            {
                perror("Shared memory create");
                ROS_INFO("Shared memory create");
            //   SharedMemoryFree();
            //   shmid = shmget((key_t)KEY_NUM, MEM_SIZE, IPC_CREAT| 0666);
                
            //   if(shmid == -1)
            //   {
            //       perror("Shared memory create fail");
            //       return 1;
            //   }
            }
        }*/
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
    else if (tc.mode == 11)
    {
        if (tc.walking_enable == 1.0)
        {
            if (walking_tickc > 0)
            {
                if (contactModec == 1.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if (contactModec == 2.0)
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
                if (contactModec == 1.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if (contactModec == 2.0)
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

            if (dc_.mode == "realrobot")
            {
                TorqueGrav(1) = 1.2 * TorqueGrav(1);
                TorqueGrav(7) = 1.2 * TorqueGrav(7);
            }

            if (walking_tickc > 0)
            {
                if (phaseChangec == true && phaseChangec1 == false)
                {
                    rate = DyrosMath::cubic(walking_tickc, double2Single_prec, double2Singlec, 1, 0, 0, 0);
                    TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(current_step_numc, 6));
                }
                else if (phaseChangec == false && phaseChangec1 == true)
                {
                    rate = DyrosMath::cubic(walking_tickc, single2Double_prec, single2Doublec, 0, 1, 0, 0);
                    if (current_step_numc < total_step_numc - 1)
                        TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(current_step_numc, 6));
                    else
                        TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(total_step_numc - 1, 6));
                }
                else
                {
                    rate = 1.0;
                    TorqueContact = wbc_.contact_force_redistribution_torque_walking(rd_, TorqueGrav, fc_redis, fc_ratio, rate, foot_stepc(current_step_numc, 6));
                }
            }

            /*
            mtx_wlk.lock();
            for(int i=0; i<MODEL_DOF; i++)
            {
                dc_.torque_dist(i) = m_gain(i)*torque_dis(i);
            }
             mtx_wlk.unlock();*/

            for (int i = 0; i < 12; i++)
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
        else if (tc.walking_enable == 3.0)
        {
            rd_.ee_[0].contact = 1.0;
            rd_.ee_[1].contact = 1.0;

            TorqueGrav = wbc_.gravity_compensation_torque(rd_);
            rd_.torque_grav_cc = TorqueGrav;
        }
    }
        //    file[0] << walking_tickc<<"\t"<<rate<<"\t" << dc_.torque_desired(3) << "\t" << double2Single_prec<< "\t"<<single2Double_prec<<"\t"<<wkc_.RF_trajectory_float.translation()(2) << "\t"<<wkc_.LF_trajectory_float.translation()(2) << "\t"<<phaseChangec<<"\t"<<phaseChangec1<<std::endl;      
     //   file[0] << dc_.torque_desired(1)<<"\t"<<dc_.torque_desired(2)<<"\t" << dc_.torque_desired(3) << "\t" << dc_.torque_desired(4)<< "\t"<<dc_.torque_desired(5)<<"\t"<<wkc_.RF_trajectory_float.translation()(2) << "\t"<<wkc_.LF_trajectory_float.translation()(2) << "\t"<<phaseChangec<<"\t"<<phaseChangec1<<std::endl;      
}

void CustomController::computeFast()
{
    if (tc.mode == 10)
    {
    }
    else if (tc.mode == 11)
    {
        if (tc.walking_enable == 1.0)
        {
        }
    }
}

void CustomController::computePlanner()
{
    for (int i = 0; i < 6; i++)
    {
        joint_state_msg.position[i] = dc_.q_virtual_(i);
        joint_state_msg.velocity[i] = dc_.q_dot_virtual_(i);
    //    joint_state_msg.effort[i] = dc_.q_ddot_virtual_(i);
    }

    joint_state_msg.position[6] = dc_.q_virtual_(39);
  //  joint_state_msg.effort[6] = 0.0;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        joint_state_msg.position[i + 7] = dc_.q_virtual_(i + 6);
        joint_state_msg.velocity[i + 7] = dc_.q_dot_virtual_(i + 6);
  //      joint_state_msg.effort[i + 7] = dc_.q_ddot_virtual_(i + 6);
    }

    joint_pin_pub.publish(joint_state_msg);

    if (callback_check == true)
    {
        if (tc.mode == 11)
        {
            //task controller for mode 11 ....
            int walkingHz = 1000;
            std::chrono::high_resolution_clock::time_point t_finish;
            std::chrono::duration<double> e_s[1];
            std::chrono::high_resolution_clock::time_point t[4];
            if (tc.walking_enable == 1.0)
            {
                t[0] = std::chrono::high_resolution_clock::now();

                if (command_init == true)
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

                Eigen::VectorQd q_ddot_est_lpf;

                jointVelocityEstimate1();

                if(wkc_.walking_tick == 0)
                {
                    q_ddot_est1.setZero();
                    q_dot_est1_prev.setZero();
                    q_dot_est1_pprev.setZero();
                }
                else
                {
                    q_ddot_est1 = (q_dot_est1 - q_dot_est1_prev)*(1000);
                    q_ddot_est_lpf = DyrosMath::lpf(q_ddot_est1, q_dot_est1_pprev,1000,200);
                }

                q_dot_est1_pprev = q_ddot_est1;
                q_dot_est1_prev = q_dot_est1;

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

                std_msgs::String sim_msg_;
                if (wkc_.walking_tick > 5300 && wkc_.walking_tick < 5500)
                {
                    sim_msg_.data = "DIST";
                //    dist_pub.publish(sim_msg_);
                }
                else if (wkc_.walking_tick == 5500)
                {
                    sim_msg_.data = "NONE";
                //    dist_pub.publish(sim_msg_);
                }

                for (int i = 0; i < 12; i++)
                {
                    ControlVal_(i) = wkc_.desired_leg_q(i);
                }
                for (int i = 12; i < MODEL_DOF; i++)
                {
                    ControlVal_(i) = wkc_.desired_init_leg_q(i);
                }

                ControlVal_(12) = wkc_.q_w(0);
                ControlVal_(13) = wkc_.q_w(1);
                ControlVal_(14) = wkc_.q_w(2);
                ControlVal_(16) = wkc_.q_w(3);
                ControlVal_(26) = wkc_.q_w(4);

                rd_.q_dot_desired_.setZero();

                if (wkc_.walking_tick == 1)
                {
                    contactforce_lpf.segment<6>(0) = rd_.ContactForce_FT.segment<6>(0);
                    contactforce_lpf.segment<6>(6) = rd_.ContactForce_FT.segment<6>(6);
                }
                else
                {
                    for (int i = 0; i < 12; i++)
                    {
                        contactforce_lpf(i) = DyrosMath::lowPassFilter(rd_.ContactForce_FT(i), contactforce_prev(i), 1.0 / 1000, 0.02);
                    }
                }

                contactforce_prev = contactforce_lpf;
                Eigen::VectorQd q_ddot_est;

                //MOB & ZMP control START

                Eigen::Vector3d desired_ankle_torque, pr, pl, ZMP_ft, com_moment, zmp2com, comExtF, comExtF_debug;
                Eigen::Vector2d x_temp, L_temp, cmp, com_accref;
                double L__, L_temp1, A__;
                Eigen::Vector2d RT, LT;
                double kp = 0.6;

                pr(2) = 0.0;
                pl(2) = 0.0;
                ZMP_ft(2) = 0.0;

                for (int i = 0; i < 2; i++)
                {
                    pr(i) = rd_.link_[Right_Foot].xipos(i);
                    pl(i) = rd_.link_[Left_Foot].xipos(i);
                    ZMP_ft(i) = rd_.ZMP_ft(i);
                }

                A__ = (pr(1) - pl(1)) / (pr(0) - pl(0));
                L_temp1 = sqrt((pr(1) - pl(1)) * (pr(1) - pl(1)) + (pr(0) - pl(0)) * (pr(0) - pl(0)));

                Eigen::Vector3d tt__;
                tt__(2) = 0;
                if (wkc_.current_step_num != wkc_.total_step_num && wkc_.walking_tick < wkc_.t_total + wkc_.t_last - 3)
                {
                    tt__(0) = wkc_.zmp_refx(wkc_.walking_tick);
                    tt__(1) = wkc_.zmp_refy(wkc_.walking_tick);
                }
                else
                {
                    tt__(0) = wkc_.zmp_refx(wkc_.t_total + wkc_.t_last - 4);
                    tt__(1) = wkc_.zmp_refy(wkc_.t_total + wkc_.t_last - 4);
                }
                L_temp1 = sqrt((pr(1) - pl(1)) * (pr(1) - pl(1)) + (pr(0) - pl(0)) * (pr(0) - pl(0)));

                x_temp(0) = (A__ * pl(0) + tt__(0) / A__ + tt__(1) - pl(1)) / (A__ + 1 / A__);
                x_temp(1) = A__ * (x_temp(0) - pl(0)) + pl(1);

                if (wkc_.contactMode == 1.0)
                {
                    desired_ankle_torque = -DyrosMath::skew((pr - tt__)) * rd_.ContactForce_FT.segment<3>(6) - DyrosMath::skew((pl - tt__)) * rd_.ContactForce_FT.segment<3>(0);
                }
                else if (wkc_.contactMode == 2.0)
                {
                    desired_ankle_torque = -DyrosMath::skew((pr - tt__)) * rd_.ContactForce_FT.segment<3>(6);
                }
                else
                {
                    desired_ankle_torque = -DyrosMath::skew((pl - tt__)) * rd_.ContactForce_FT.segment<3>(0);
                }

                RT = L__ * desired_ankle_torque.segment<2>(0);
                LT = (1 - L__) * desired_ankle_torque.segment<2>(0);

                double L1 = 50;

                if(wkc_.walking_tick == 1)
                {
                    torque_dis_prev.setZero();
                }

                Int_dis += (dc_.tocabi_.Cor_.transpose() *q_dot_est1/* q_dot_est1/* rd_.q_dot_virtual_.segment<MODEL_DOF>(6)*/ - dc_.tocabi_.G_ + dc_.torque_desired + torque_dis) / 1000.0;

                torque_dis = L1 * (dc_.tocabi_.M_p * q_dot_est1/* rd_.q_dot_virtual_.segment<MODEL_DOF>(6)*/ - Int_dis);

                torque_dis = 0.7 * torque_dis + 0.3 * torque_dis_prev;
                torque_dis_prev = torque_dis;

                Eigen::Vector4d toruqe_dis1, torque_dis1_lpf;

                toruqe_dis1(0) = /*LT(1) + torque_dis(4); */0.7*  LT(1) + kp * (LT(1) - (dc_.torque_desired(4)) + torque_dis(4));
                toruqe_dis1(1) = /*LT(0) + torque_dis(5); */0.7 * LT(0) + kp * (LT(0) - (dc_.torque_desired(5)) + torque_dis(5));
                toruqe_dis1(2) = /*RT(1)+ torque_dis(10); */0.7 * RT(1) + kp * (RT(1) - (dc_.torque_desired(10)) + torque_dis(10));
                toruqe_dis1(3) = /*RT(0)+ torque_dis(11); */0.7 * RT(0) + kp * (RT(0) - (dc_.torque_desired(11)) + torque_dis(11));

                if (wkc_.walking_tick == 1.0)
                {
                    torque_dis1_prev = toruqe_dis1;
                }

                for (int i = 0; i < 4; i++)
                {
                    torque_dis1_lpf(i) = DyrosMath::lowPassFilter(toruqe_dis1(i), torque_dis1_prev(i), 1.0 / 1000, 0.05); //0.2
                }

                torque_dis1_prev = torque_dis1_lpf;

                mtx_wlk.lock();

                dc_.torque_dist.setZero();

                dc_.torque_dist(4) = torque_dis1_lpf(0);  // /*LT(1) + torque_dis(4);*/0.3* LT(1)+kp*(LT(1) -(dc_.torque_desired(4)) + torque_dis(4));
                dc_.torque_dist(5) = torque_dis1_lpf(1);  ///*LT(0) + torque_dis(5);*/0.3*LT(0)+kp*(LT(0) -(dc_.torque_desired(5)) + torque_dis(5));
                dc_.torque_dist(10) = torque_dis1_lpf(2); // /*LT(1) + torque_dis(4);*/0.3* LT(1)+kp*(LT(1) -(dc_.torque_desired(4)) + torque_dis(4));
                dc_.torque_dist(11) = torque_dis1_lpf(3); ///*LT(0) + torque_dis(5);*/0.3*LT(0)+kp*(LT(0) -(dc_.torque_desired(5)) + torque_dis(5));

                if (wkc_.contactMode == 2)
                {
                    dc_.torque_dist(10) = 0.0;
                    dc_.torque_dist(11) = 0.0;
                }
                else if (wkc_.contactMode == 3)
                {
                    dc_.torque_dist(4) = 0.0;
                    dc_.torque_dist(5) = 0.0;
                }

                mtx_wlk.unlock();

                //MOB & ZMP END
                
                Eigen::Vector2d ZMP_lpf;
                Eigen::Vector3d com_ddot;

                if(wkc_.walking_tick == 1)
                {
                    zmp_prevlpf = ZMP_ft.segment<2>(0);
                    ZMP_lpf= ZMP_ft.segment<2>(0);
                    com_ddot.setZero();
                }
                else
                {
                    for(int i = 0; i <2; i++)
                    {
                        ZMP_lpf(i) = DyrosMath::lowPassFilter(ZMP_ft(i),zmp_prevlpf(i),1.0/1000 ,0.05);
                    }
                    com_ddot = (rd_.com_.vel - com_dotprev)*1000;
                }

                zmp_prevlpf = ZMP_lpf;

                if(wkc_.walking_tick > 300)
                    q_ddot_est = dc_.A_inv.block<MODEL_DOF,MODEL_DOF>(6,6)*(dc_.torque_desired - dc_.tocabi_.Cor_*(dc_.q_dot_virtual_.segment<MODEL_DOF>(6))/*q_dot_est1*/ - dc_.tocabi_.G_ + torque_dis);



                /*    Eigen::Vector3d cmp;
                if(contactModec == 1)
                {
                    cmp(1) = 0.0;
                }
                else if(contactModec == 2)
                { // LeFt support
                    cmp(1) = rd_.com_.pos(1) -(rd_.ContactForce_FT(1))/(rd_.ContactForce_FT(2))*wkc_.zc;
                }
*                else
                {
                    cmp(1) = rd_.com_.pos(1) -(rd_.ContactForce_FT(7))/(rd_.ContactForce_FT(8))*wkc_.zc;
                }*/

                if (walking_tickc > 0)
                {
                    //           file[1] << comExtF(0) << "\t" << comExtF(1) <<"\t" << comExtF_debug(0) <<"\t" <<comExtF_debug(1)<<std::endl;

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
        //        file[1] << q_ddot_est(3)<<"\t"<<dc_.q_ddot_virtual_(9) << "\t" <<q_ddot_est(5)<<"\t"<< dc_.q_ddot_virtual_(11) << std::endl;
                //file[1] <<torque_dis(4)+dc_.torque_desired(4) << "\t" << dc_.q_ddot_virtual_(10) << std::endl;
                
           //     file[1] <<wkc_.comFcur(0)<<"\t"<<wkc_.comFref(0)<<"\t"<<wkc_.CM_moment_lpf(1) <<"\t" << wkc_.CM_moment_lpf(2) << "\t" << wkc_.CM_moment_lpf(0) <<"\t"<<rd_.com_.angular_moment(0)<<"\t"<<rd_.com_.angular_moment(1)<<"\t"<<rd_.com_.angular_moment(2)<< std::endl;
                
                //////////////1209///  
            //    file[1] << rd_.com_.accel(0) << "\t"<< rd_.com_.accel(1) << "\t"<< rd_.com_.accel(2) << "\t" <<com_accref(0)<<"\t"<<com_accref(1)<<std::endl;    
//file[1] << rd_.q_dot_virtual_(6) << "\t"<< rd_.q_dot_virtual_(7) << "\t"<< rd_.q_dot_virtual_(8) << "\t"<< rd_.q_dot_virtual_(9)<<"\t"<<rd_.q_dot_est(0) << "\t"<<rd_.q_dot_est(1) << "\t"<<rd_.q_dot_est(2) << "\t"<<rd_.q_dot_est(3) << "\t" <<std::endl;

         //          file[0] << q_dot_est1(0) << "\t" << rd_.q_dot_virtual_(6) <<"\t"<< q_dot_est1(1) << "\t" << rd_.q_dot_virtual_(7) <<"\t"<< q_dot_est1(2) << "\t" << rd_.q_dot_virtual_(8) <<"\t"<< q_dot_est1(3) << "\t" << rd_.q_dot_virtual_(9) <<"\t"<< q_dot_est1(4) << "\t" << rd_.q_dot_virtual_(10) <<"\t"<< q_dot_est1(5) << "\t" << rd_.q_dot_virtual_(11) <<std::endl;
          //         file[1] << q_ddot_est_lpf(0) << "\t" << rd_.q_ddot_virtual_(6)<<"\t"<< q_ddot_est_lpf(1) << "\t" << rd_.q_ddot_virtual_(7)<<"\t"<< q_ddot_est_lpf(2) << "\t" << rd_.q_ddot_virtual_(8)<<"\t"<< q_ddot_est_lpf(3) << "\t" << rd_.q_ddot_virtual_(9)<<"\t"<< q_ddot_est_lpf(4) << "\t" << rd_.q_ddot_virtual_(10)<<"\t"<< q_ddot_est1(5) << "\t" << rd_.q_ddot_virtual_(11)<<std::endl; 

                   
                  // file[1] << wkc_.desired_leg_q_dot(0) << "\t" << rd_.q_dot_(0) <<"\t"<< wkc_.desired_leg_q_dot(1) << "\t" << rd_.q_dot_(1) <<"\t"<< wkc_.desired_leg_q_dot(3) << "\t" << rd_.q_dot_(3) <<"\t"<<rd_.q_dot_virtual_(9)<< std::endl;
                 //   file[1] <<wkc_.desiredH_leg(0)<<"\t"<<wkc_.desiredH_leg(1)<<"\t"<< wkc_.CM_momentum_prev(0) << "\t" <<wkc_.CM_momentum_prev(1) << "\t" <<wkc_.CM_momentum_prev(2) << "\t"<< wkc_.CM_momentum_prev1(0) << "\t" <<wkc_.CM_momentum_prev1(1) << "\t" <<wkc_.CM_momentum_prev1(2) << "\t"<<wkc_.desiredH_comp(0)<<"\t"<<wkc_.desiredH_comp(1)<<"\t"<<wkc_.desiredH_comp(2)<<"\t"<<wkc_.comFerr(0)<<"\t"<<wkc_.comFerr(1)<<"\t"<<wkc_.comFerr(2)<<std::endl;
//file[1] << wkc_.desiredH_comp(0) << "\t" << wkc_.CM_moment_lpf(0) << std::endl;
                   //file[1] <<wkc_.comFerr(0) << "\t" << wkc_.comFerr(1) << "\t" << wkc_.comFcur(0) <<"\t" << wkc_.comFcur(1) << "\t" << wkc_.comFref(0)<<"\t" << wkc_.comFref(1) << std::endl;
                   //  file[1]<<ZMP_ft(0) -0.03<< "\t"<<ZMP_ft(1)<<"\t"<<wkc_.zmp_refx(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refy(wkc_.walking_tick)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t" << wkc_.com_refx(wkc_.walking_tick)<<"\t" << wkc_.com_refy(wkc_.walking_tick) <<"\t"<<rd_.com_.pos(0)-0.0287<<"\t"<<rd_.com_.pos(1)<<"\t"<<rd_.com_.pos(2)<<"\t"<<wkc_.PELV_trajectory_float.translation()(0)<<"\t"<<wkc_.PELV_trajectory_float.translation()(1)<<std::endl;

                //file[1]<<rd_.q_(4)<<"\t"<<wkc_.desired_leg_q(4)<<"\t"<<rd_.com_.pos(0)-0.03<<"\t"<<rd_.com_.pos(1)<<"\t"<<rd_.ZMP_ft(0)<<"\t"<<rd_.ZMP_ft(1)<<"\t"<<ZMP_lpf(0) - 0.03 <<"\t" << ZMP_lpf(1) <<"\t"<<wkc_.zmp_refx(wkc_.walking_tick)<<"\t"<<wkc_.zmp_refy(wkc_.walking_tick)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t"<<pl(1)<<"\t"<<pr(1)<<std::endl;
                    //     file[1] <<ZMP_ft(0) <<"\t"<<rd_.com_.pos(0) <<"\t"<<rd_.com_.angular_moment(1)<<"\t"<<cmp(0)<<"\t" << cmp(1) <<std::endl;//"\t" << cmp_t(0) << "\t" << cmp_t(1) << std::endl;

                //     file[1] <<wkc_.PELV_trajectory_float.translation()(0)<<"\t"<<wkc_.PELV_trajectory_float.translation()(1)<<"\t"<< wkc_.com_refy(wkc_.walking_tick) << "\t" << rd_.com_.pos(1)<<"\t" << rd_.link_[Pelvis].xipos(1) <<"\t" << wkc_.xy_vib_est(0)<<std::endl;
                //   "\t" << rd_.link_[Left_Foot].xipos(0) << "\t" << rd_.link_[Left_Foot].xipos(1) << "\t" << rd_.link_[Left_Foot].xipos(2)<< "\t" << rd_.link_[Right_Foot].xipos(0) << "\t" << rd_.link_[Right_Foot].xipos(1)<< "\t" << rd_.link_[Right_Foot].xipos(2)
                //file[1]<<dc_.torque_dist(4)<<"\t"<<dc_.torque_dist(5)<<"\t"<<dc_.torque_dist(10)<<"\t"<<dc_.torque_dist(11)<<"\t"<< L__<<"\t"<<ZMP_ft(0)<<"\t"<<ZMP_ft(1)<<"\t"<< L__ <<"\t" <<desired_ankle_torque(0) <<"\t" << desired_ankle_torque(1) <<"\t" << desired_ankle_torque(2) <<std::endl;
            //    file[1] << wkc_.comFcur(0) << "\t"<< wkc_.comFcur(1) << "\t"<< wkc_.comFcur(2) << "\t"<< wkc_.comFref(0) << "\t"<< wkc_.comFref(1) << "\t"<< wkc_.comFref(2) << "\t"<< wkc_.comFerr(0) << "\t"<< wkc_.comFerr(1) << "\t" << wkc_.CM_momentumUpperd_(0) <<"\t" << wkc_.CM_momentumUpperd_(1) <<"\t"<<wkc_.CM_momentUpper_(0)<<"\t"<<wkc_.CM_momentUpper_(1)<<"\t"<<wkc_.H_leg(0)<<"\t"<<wkc_.H_leg(1)<<"\t"<<wkc_.q_dm(0)<<"\t"<<wkc_.q_dm(1)<<"\t"<<wkc_.q_dm(2)<<"\t"<<rd_.com_.pos(0)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<rd_.com_.pos(1)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t"<<wkc_.Hl_leg(0)<<"\t"<<wkc_.Hl_leg(1)<<"\t"<<wkc_.Hl_leg(2)<<"\t"<<wkc_.com_refdx(wkc_.walking_tick)<<"\t"<<rd_.com_.vel(0)<<std::endl;
          //file[1] << wkc_.desired_leg_q(2) << "\t"<< wkc_.desired_leg_q(8) << "\t"<< wkc_.desired_leg_q(3) << "\t"<< wkc_.desired_leg_q(9) << "\t"<< wkc_.desired_leg_q(4) << "\t"<< wkc_.desired_leg_q(10) <<std::endl;  
        //    file[1] <<wkc_.H_leg(0) << "\t" << wkc_.H_leg(1) << "\t" << wkc_.H_leg(2) << "\t" <<  wkc_.H_legl(0) << "\t" << wkc_.H_legl(1) << "\t" << wkc_.H_legl(2) << "\t" << wkc_.H_legr(0) << "\t" << wkc_.H_legr(1) << "\t" << wkc_.H_legr(2) << "\t"<<rd_.q_dot_(2) <<"\t"<<rd_.q_dot_(8) <<"\t"<<rd_.q_dot_(3) <<"\t"<<rd_.q_dot_(9) << "\t"<<rd_.q_dot_(4) <<"\t"<<rd_.q_dot_(10) <<"\t"<<wkc_.q_w(0)<<"\t"<<wkc_.q_w(1)<<"\t"<<wkc_.q_w(2)<<"\t"<<wkc_.com_refx(wkc_.walking_tick)<<"\t"<<rd_.com_.pos(0)<<"\t"<<rd_.link_[Right_Foot].xipos(0) << "\t" << rd_.link_[Left_Foot].xipos(0)<<std::endl;


         /*   if(wkc_.walking_tick == 1)
            {
                for(int i = 0; i < 3; i++)
                {
                    for(int j = 0; j < 3; j++)
                    {
                        file[1] << wkc_.Ag_waist(i, j)<<"\t";
                    }
                }

         /*       
                for(int i = 0; i < 3; i++)
                {
                    file[1] << wkc_.Ag_armL(i, 1)<<"\t"<< wkc_.Ag_armR(i, 1)<<"\t";
                }

                file[1] << std::endl;
            }*/
       file[1] << wkc_.H_leg1(0)<<"\t"<<wkc_.H_leg1(1)<<"\t"<<wkc_.H_leg1(2)<<std::endl;
         
        
                t[1] = std::chrono::high_resolution_clock::now();
                e_s[0] = t[0] - t[1];
            }
            else if (tc.walking_enable == 3.0)
            {
                if (command_init == true)
                {
                    cycle_count = 0;
                    command_init = false;
                    wkc_.setRobotStateInitialize(rd_);
                    wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.walking_pattern2, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, tc.imu_walk, tc.mom, tc.vibration_control, rd_);
                    t_begin = std::chrono::high_resolution_clock::now();
                }

                wkc_.walkingCompute(rd_);

                rd_.q_dot_desired_.setZero();

                for (int i = 0; i < MODEL_DOF; i++)
                {
                    ControlVal_(i) = wkc_.desired_init_leg_q(i);
                }
            }
        }
    }
    else
    {
        ControlVal_ = rd_.q_init_;
    }
}

void CustomController::PinocchioCallback(const tocabi_controller::model &msg)
{
    a++;
    if (walking_tickc == 0)
    {
        std::cout << "AG_" << std::endl;

        std::cout << dc_.tocabi_.Ag_ << std::endl;
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < MODEL_DOF; j++)
        {
            dc_.tocabi_.Ag_(i, j) = msg.CMM[33 * i + j];
        }
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 0; j < MODEL_DOF; j++)
        {
            dc_.tocabi_.Cor_(i, j) = msg.COR[33 * i + j];
        }
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 0; j < MODEL_DOF; j++)
        {
            dc_.tocabi_.M_p(i, j) = msg.M[33 * i + j];
        }
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc_.tocabi_.G_(i) = msg.g[i];
    }

/*    for (int i = 0; i < 3; i++)
    {
        CM_moment_pin(i) = msg.Cmoment[i];
    }
*/
    dc_.tocabi_.M_p.triangularView<Eigen::StrictlyLower>() = dc_.tocabi_.M_p.transpose().triangularView<Eigen::StrictlyLower>();

    if (callback_check == false)
    {
        callback_tick++;

        if (callback_tick > 50 && callback_check == false)
        {
            callback_check = true;
        }
    }
}

void CustomController::jointVelocityEstimate1()
{
    //Estimate joint velocity using state observer
    double dt;
    dt = 1 / 1000;
    Eigen::MatrixXd A_t, A_dt, B_t, B_dt, C, I, I_t;
    I.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    I.setIdentity();
    I_t.setZero(MODEL_DOF, MODEL_DOF);
    I_t.setIdentity();
    A_t.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    A_dt.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    B_t.setZero(MODEL_DOF * 2, MODEL_DOF);
    B_dt.setZero(MODEL_DOF * 2, MODEL_DOF);
    C.setZero(MODEL_DOF, MODEL_DOF * 2);

    A_t.topRightCorner(MODEL_DOF, MODEL_DOF);
    A_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = dc_.A_inv.bottomRightCorner(MODEL_DOF, MODEL_DOF) * dc_.tocabi_.Cor_;
    A_t.topRightCorner(MODEL_DOF, MODEL_DOF) = I_t;
    B_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = dc_.A_inv.bottomRightCorner(MODEL_DOF, MODEL_DOF);
    C.bottomLeftCorner(MODEL_DOF, MODEL_DOF) = I_t * dt;
    B_dt = B_t * dt;
    A_dt = I - dt * A_dt;

    double L, L1;
    L = 0.15;
    L1 = 0.1;

    if (velEst == false)
    {
        q_est1 = rd_.q_;
        q_dot_est1 = rd_.q_dot_;
        velEst = true;
    }

    if (velEst == true)
    {
        Eigen::VectorQd q_temp;
        Eigen::VectorVQd q_dot_virtual;

        //  q_dot_virtual = rd_.q_dot_virtual_;

        q_temp = q_est1;

        if (debug == false)
        {
            std::cout << "q" << std::endl;
            std::cout << rd_.q_ << std::endl;
            std::cout << "dc_.tocabi_.Cor_" << std::endl;
            std::cout << q_est1 << std::endl;
            std::cout << "q_dot_est1" << std::endl;
            std::cout << dt * q_dot_est1 << std::endl;
            std::cout << "dc_.tocabi_.G_" << std::endl;
            std::cout << L * (rd_.q_ - q_est1) << std::endl;
        }

        q_est1 = q_est1 + dt * q_dot_est1 + L * (rd_.q_ - q_est1);

        q_dot_virtual.segment<MODEL_DOF>(6) = q_dot_est1;

        q_dot_est1 = (q_temp - q_est1) * 1000.0;

        Eigen::VectorQd tau_;

        tau_ = dc_.tocabi_.Cor_ * q_dot_est1 + dc_.tocabi_.G_;

        debug = true;

        q_dot_est1 = -(q_dot_est1 + B_dt.bottomRightCorner(MODEL_DOF, MODEL_DOF) * (dc_.torque_desired + L1 * (rd_.q_ - q_est1) - tau_));
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
     20.0, 20.0,*
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
