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
        int walkingHz = 1500;  
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

             // for (size_t i = 1; i < LINK_NUMBER; i++) 
             // {
    //I_temp[i] = dc_.tocabi_.link_[1].model->I[i].toMatrix();
    //model.hc[i] = model.Ic[i].toMatrix() * model.v[i];
              //}



          /*  for(int i = LINK_NUMBER-1; i >0; i--)
            {
                if(dc_.tocabi_.link_[1].model->lambda[i] == 0)
                {
                    temp11 = temp11 + dc_.tocabi_.link_[1].model->X_lambda[i].applyTranspose(temp13);
                    temp12 = temp12 + dc_.tocabi_.link_[1].model->X_lambda[i].applyTranspose(dc_.tocabi_.link_[1].model->hc[i]);
                }
                else
                {
                    temp13 = dc_.tocabi_.link_[1].model->I[i] * dc_.tocabi_.link_[1].model->v[i];
                }
            }*/

          /*  for(int i = LINK_NUMBER-1; i >0; i--)
            {
                if(dc_.tocabi_.link_[1].model->lambda[i] == 0)
                {
                    temp11 = temp11 + dc_.tocabi_.link_[1].model->X_lambda[i].applyTranspose(temp13);
                    temp12 = temp12 + dc_.tocabi_.link_[1].model->X_lambda[i].applyTranspose(dc_.tocabi_.link_[1].model->hc[i]);
                }
                else
                {
                    temp13 = dc_.tocabi_.link_[1].model->I[i] * dc_.tocabi_.link_[1].model->v[i];
                }
            }

            temp11 = dc_.tocabi_.link_[1].spatialTransform * temp11;
            temp12 = dc_.tocabi_.link_[1].spatialTransform * temp12;*/
/*
std::cout << "I" << std::endl;
            
          //  std::cout << dc_.tocabi_.link_[1].model->Ic[5] * dc_.tocabi_.link_[1].model->v[5] <<std::endl;
  std::cout << temp11 <<std::endl;

std::cout << "h" << std::endl;
          //  std::cout << dc_.tocabi_.link_[1].model->hc[5] <<std::endl;
        
          std::cout <<temp12  <<std::endl;
  */      
        /*  Eigen::Matrix6d dd;
          dc_.tocabi_.I1.resize(6, INERITA_SIZE);
        
            for(int i = 1; i < LINK_NUMBER; i++)
            {
                dd.topLeftCorner(3,3) = dc_.tocabi_.link_[i].model->Ic[i].toMatrix().bottomRightCorner(3,3);
                dd.topRightCorner(3,3) = dc_.tocabi_.link_[i].model->Ic[i].toMatrix().bottomLeftCorner(3,3);
                dd.bottomLeftCorner(3,3) = dc_.tocabi_.link_[i].model->Ic[i].toMatrix().topRightCorner(3,3);
                dd.bottomRightCorner(3,3) = dc_.tocabi_.link_[i].model->Ic[i].toMatrix().topLeftCorner(3,3);
            
                dc_.tocabi_.I1.block<6,6>(0,6*(i-1)) = dd;
            }
*/

          /*  std::cout << rd_.link_[2].model->X_base[1].toMatrix() <<std::endl;
std::cout << "MODEL" <<std::endl;
            
            std::cout << rd_.link_[1].spatialTransform <<std::endl;
*/

            Eigen::Vector6d mom_pino;
            mom_pino = dc_.tocabi_.Ag_ * dc_.tocabi_.q_dot_;
            wkc_.walkingCompute(rd_);
        //    file[0] << dc_.tocabi_.ZMP_ft(0) << "\t" <<  dc_.tocabi_.ZMP_ft(1) << "\t" << wkc_.com_refx(wkc_.walking_tick)<<"\t"<<wkc_.com_refy(wkc_.walking_tick)<<"\t"<<wkc_.PELV_trajectory_float.translation()(2)<<"\t"<<wkc_.PELV_trajectory_float.translation()(1)<<std::endl;
        //    file[1] << dc_.tocabi_.ZMP(0) << "\t"<< dc_.tocabi_.ZMP(1) << "\t"<<dc_.tocabi_.com_.pos(0) << "\t"<< dc_.tocabi_.com_.pos(1) <<"\t" << dc_.tocabi_.link_[Pelvis].xpos(0) <<"\t"<<wkc_.debug<<std::endl;
        //      file[0] << dc_.tocabi_.ZMP(0)<<"\t"<< wkc_.com_refx(wkc_.walking_tick) << "\t" << wkc_.com_refy(wkc_.walking_tick) << "\t" << wkc_.RFDotTraj(0) <<"\t"<< wkc_.RFDotTraj(1)  <<"\t"<< wkc_.RFDotTraj(2)  <<"\t"<< wkc_.aa(2) << "\t" << wkc_.aa(3) << "\t" << wkc_.aa(4) << "\t" << wkc_.aa(5) << "\t"<< wkc_.aa(6) << "\t" << wkc_.aa(7) << "\t" << wkc_.aa(8) << "\t" << wkc_.aa(9) << "\t" << wkc_.aa(10) << "\t" << wkc_.aa(11) << std::endl;
        //      file[1] << DyrosMath::rot2Euler((rd_.link_[Left_Foot].Rotm))(1)<<"\t"<< wkc_.RF_trajectory_float.translation()(0) << "\t" << wkc_.RF_trajectory_float.translation()(1) << "\t" << wkc_.LF_trajectory_float.translation()(0) << "\t" << wkc_.RF_float_current.translation()(0) << "\t" << wkc_.RF_float_current.translation()(1) << "\t"<< wkc_.RF_float_current.translation()(2) << "\t"  <<wkc_.Cfsemd(0) << "\t" << wkc_.Cfsemd(1) << "\t"<< wkc_.Cfsemd(2) << "\t" << wkc_.desired_leg_q(7) << "\t" << wkc_.desired_leg_q(8) << "\t" <<wkc_.desired_leg_q(9) << "\t" <<wkc_.desired_leg_q(10) << "\t" <<wkc_.desired_leg_q(11) << std::endl;
       file[0] << rd_.com_.vel(0) << "\t" << rd_.com_.vel(1) << "\t"<< rd_.com_.vel(2) << "\t" <<wkc_.MT_cen(0)/95.1703 << "\t"<<wkc_.MT_cen(1)/95.1703 << "\t"<<wkc_.MT_cen(2)/95.1703 << std::endl;
       //      file[0] << dc_.tocabi_.ZMP(0)<<"\t"<< wkc_.aa(0) << "\t" << wkc_.aa(1) << "\t" << wkc_.aa(2) << "\t" << wkc_.aa(3) << "\t" << wkc_.aa(4) << "\t" << 
    //    wkc_.aa(5) << std::endl;

   file[1] <<mom_pino(0) <<"\t"<<mom_pino(1) <<"\t"<<mom_pino(2) <<"\t"<<rd_.com_.angular_momentum(0) <<"\t"<<rd_.com_.angular_momentum(1) <<"\t"<<rd_.com_.angular_momentum(2)<<std::endl;
   //     file[1] << rd_.com_.angular_momentum(0) <<"\t" << rd_.com_.angular_momentum(1)<<"\t" << rd_.com_.angular_momentum(2) << "\t"<<wkc_.MT_cen(3) << "\t"<<wkc_.MT_cen(4)<< "\t"<<wkc_.MT_cen(5)<<"\t" <<wkc_.MT_cen(0) <<"\t"<< wkc_.MT_cen(1) <<"\t"<< wkc_.MT_cen(2) <<"\t"   <<std::endl;// "\t" << wkc_.q_w<< std::endl;
       //  file[0] << wkc_.walking_tick<< "\t"<<wkc_.desired_leg_q(1)<< std::endl;
         //   file[0]<< wkc_.COM(1) <<"\t"<<wkc_.COM_support_current.translation()(1) << "\t"<<wkc_.com_support_temp(1) <<"\t" << wkc_.capturePoint_refy(wkc_.walking_tick) <<"\t" << wkc_.com_refy(wkc_.walking_tick) << std::endl;
         //   file[0] <<wkc_.desired_leg_q(0) <<"\t"<<rd_.q_(0)<<"\t"<<wkc_.desired_leg_q(1) <<"\t"<<rd_.q_(1)<<"\t"<<wkc_.desired_leg_q(2) <<"\t"<<rd_.q_(2)<<"\t"<<wkc_.desired_leg_q(3) <<"\t"<<rd_.q_(3)<<"\t"<<wkc_.desired_leg_q(4) <<"\t"<<rd_.q_(4)<<"\t"<<wkc_.desired_leg_q(5)<<"\t" <<rd_.q_(5)<<std::endl; 
         //    file[0] <<DyrosMath::rot2Euler((rd_.link_[Left_Foot].Rotm))(1)<<"\t"<<DyrosMath::rot2Euler((rd_.link_[Right_Foot].Rotm)/*.inverse() * rd_.link_[Left_Foot].Rotm*/)(1) <<"\t"<<DyrosMath::rot2Euler(rd_.link_[Right_Foot].Rotm)(0)<<"\t"<<DyrosMath::rot2Euler(rd_.link_[Left_Foot].Rotm)(0) <<"\t"<<DyrosMath::rot2Euler(rd_.link_[Right_Foot].Rotm)(1)<<"\t"<<wkc_.desired_leg_q(5) <<"\t"<<rd_.q_(5)<<"\t"<<wkc_.desired_leg_q(11) <<"\t"<<rd_.q_(11)<<"\t"<<wkc_.leg_q(5)<<"\t"<<wkc_.leg_q(11)<<"\t"<<wkc_.rot_vel(0,0)<<std::endl;
        // file[0]<<rd_.q_(11)<<"\t"<<rd_.q_(12) <<"\t"<<rd_.q_(13)<<"\t"<<rd_.q_(14) <<"\t"<<rd_.q_(15)<<"\t"<<rd_.q_(16) <<"\t"<<rd_.q_(17)<<"\t"<<rd_.q_(18) <<"\t"<<rd_.q_(19)<<"\t"<<rd_.q_(20)<<"\t" <<rd_.q_(21)<<std::endl; 

                //std::cout << rd_.link_[i].model->hc[i] << std::endl;
                //std::cout << rd_.link_[i].model->Ic.toMatrix()
                //std::cout << (rd_.I * wkc_.qqqqq).segment<6>(6*i) <<std::endl;
       /*         std::cout << "---------RBDL--------" << std::endl;
            
                std::cout << rd_.link_[5].model->v[5] << std::endl;
                std::cout << "---------JAC--------" << std::endl;
                std::cout << rd_.link_[5].Jac * rd_.q_dot_virtual_ << std::endl;
                
                std::cout << "---------COMJAC--------" << std::endl;
                std::cout << rd_.link_[5].Jac_COM * rd_.q_dot_virtual_ << std::endl;
                
                std::cout << "---------v--------" << std::endl;
                
                std::cout << rd_.link_[5].v << std::endl;
                std::cout << "--------w--------" << std::endl;
                
                std::cout << rd_.link_[5].w<< std::endl;

int lambda = rd_.link_[5].model->lambda[5];
                std::cout << "--------xlamda--------" << std::endl;
std::cout << rd_.link_[5].model->X_lambda[5].apply(rd_.link_[5].model->v[lambda]) << std::endl;//rd_.link_[2].model->lambda]*///)<<std::endl;

/*              std::cout << "--------vlamda-------" << std::endl;
std::cout << rd_.link_[5].model->v_J[8] << std::endl;//rd_.link_[2].model->lambda]*///)<<std::endl;
  /*
              std::cout << "--------q-------" << std::endl;
                std::cout << rd_.link_[5].model->mJoints[5].q_index << std::endl;              

              std::cout << "--------qdot-------" << std::endl;
                std::cout << rd_.link_[5].model->mJoints[0].q_index << std::endl;              

                std::cout << "--------Jv-------" << std::endl;
std::cout << rd_.q_dot_virtual_ << std::endl;//)<<std::endl;
*/


/*std::cout <<"rbdl vdot8" << std::endl;
std::cout << rd_.link_[5].model->Ic[1].toMatrix() << std::endl;



std::cout <<"rbdl vdot5" << std::endl;
std::cout << rd_.link_[1].spatialTransform << std::endl;*/
/*
std::cout <<"Jqdot" << std::endl;
std::cout << rd_.link_[5].Jac * rd_.q_dot_virtual_<< std::endl;

std::cout <<"Jqdot" << std::endl;
std::cout << rd_.link_[5].model->v[5] - rd_.link_[5].model->v_J[5] << std::endl;

std::cout <<"Jqdot88" << std::endl;
std::cout << rd_.link_[5].model->v[8] - rd_.link_[5].model->v_J[8] << std::endl;
*/


Eigen::Vector6d temp; 
Eigen::Vector3d temp1, temp2;
Eigen::VectorVQd temp3;
Eigen::Matrix6d temp4;

temp4.setZero();
temp4.topLeftCorner(3,3).setIdentity() = rd_.link_[4].Rotm;//.transpose();
temp4.bottomRightCorner(3,3) = rd_.link_[4].Rotm;//.transpose();

temp3.setZero();
temp3.segment<MODEL_DOF>(6) = rd_.q_dot_virtual_.segment<MODEL_DOF>(6);


//std::cout << "pos" <<std::endl;
//std::cout << (rd_.link_[4].xipos - xipos_prev)*walkingHz <<std::endl;


rd_.link_[4].Get_PointPos(rd_.q_virtual_, rd_.q_dot_virtual_ , rd_.link_[4].COM_position, temp2, temp);

/*
std::cout << "pose xi" << std::endl;
std::cout << rd_.link_[4].xipos << std::endl;

std::cout << "pose temp" << std::endl;
std::cout << temp2 << std::endl;

std::cout << "rbdl" <<std::endl;
std::cout << temp <<std::endl;

std::cout << "rbdlv" <<std::endl;
std::cout << (rd_.link_[4].v) <<std::endl;

std::cout << "rbdlw" <<std::endl;
std::cout << rd_.link_[4].w <<std::endl;

*/
//ddd
/*
    std::cout << "rbdl model " << std::endl;
std::cout << (rd_.link_[4].model->v[3]) <<std::endl;

std::cout << "comJac_temp" <<std::endl;
std::cout << (temp4*rd_.link_[4].Jac_COM * rd_.q_dot_virtual_) << std::endl;

std::cout << "comJac" <<std::endl;
std::cout << (rd_.link_[4].Jac_COM * rd_.q_dot_virtual_)<<std::endl;
*/
/*
std::cout <<"rbdl" <<std::endl;
std::cout <<rd_.link_[4].model->X_base[3].toMatrix()<< std::endl;

std::cout <<"model" <<std::endl;
std::cout <<rd_.link_[4].spatialTransform << std::endl;*/

            for(int i = 0; i < 12; i++)
            {
                ControlVal_(i) = wkc_.desired_leg_q(i);
            }
            for(int i = 12; i<MODEL_DOF; i++)
            {
                ControlVal_(i) = wkc_.desired_init_leg_q(i);
            }
        //    ControlVal_(12) = wkc_.q_w(0);
        //    ControlVal_(13) = wkc_.q_w(1);
        //    ControlVal_(14) = wkc_.q_w(2);
            t[1] = std::chrono::high_resolution_clock::now();
            e_s[0]= t[0] - t[1];
          //  std::cout << "time " << e_s[0].count()*1000 << std::endl;
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