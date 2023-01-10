#include "imuPreintegrationClass.h"

ImuPreintegrationClass::ImuPreintegrationClass(const Eigen::Vector3d b_a_in, const Eigen::Vector3d b_g_in, 
    const double ACC_N, const double GYR_N, const double ACC_W, const double GYR_W){
    b_a = b_a_in;
    b_g = b_g_in;
    noise.setZero();
    noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
}

void ImuPreintegrationClass::initWithImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr){
    //reset error
    delta_p.setZero();
    delta_R.setIdentity();
    delta_v.setZero();
    jacobian.setIdentity();
    covariance.setZero();
    sum_dt = 0;
    last_acc = acc;
    last_gyr = gyr;
}

void ImuPreintegrationClass::addImuData(const double dt, const Eigen::Vector3d acc, const Eigen::Vector3d gyr){
    if(acc_buf.size()==0){
        initWithImu(acc, gyr);
    }
    propagate(dt, acc, gyr);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    dt_buf.push_back(dt);

    if (sum_dt > 0.5){
        ROS_WARN("imu time too long sum_dt = %f", sum_dt);
    }
}

void ImuPreintegrationClass::propagate(const double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr){

    Eigen::Vector3d un_acc_0 = delta_R * (last_acc - b_a);
    Eigen::Vector3d un_gyr = 0.5 * (last_gyr + gyr) - b_g;
    result_delta_R = delta_R * Utils::so3ToR(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = result_delta_R * (acc - b_a);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
    delta_v = delta_v + un_acc * dt;
    b_a = b_a;
    b_g = b_g;   

    // calculate jacobian
    Eigen::Matrix3d acc0_skew = Utils::skew(last_acc - b_a); 
    Eigen::Matrix3d acc1_skew = Utils::skew(acc - b_a);  
    Eigen::Vector3d w_x = 0.5 * (last_gyr + gyr) - b_g;
    Eigen::Matrix3d R_w_x=Utils::skew(w_x);

    double dt2 = dt * dt;
    double dt3 = dt * dt2;

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);
    F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - R_w_x * dt;
    F.block<3, 3>(0, 12) = -1.0 * Eigen::Matrix3d::Identity()* dt;
    F.block<3, 3>(3, 0) = -0.25 * result_delta_R * acc0_skew * dt2 + 
                                  -0.25 * result_delta_R * acc1_skew * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
                      
    F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 9) = -0.25 * (delta_R + result_delta_R) * dt2;
    F.block<3, 3>(3, 12) = 0.25 * result_delta_R * acc1_skew * dt3 ;
    F.block<3, 3>(6, 0) = -0.5 * delta_R * acc0_skew * dt + 
                                  -0.5 * result_delta_R*  acc1_skew  * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
    F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(6, 9) =-0.5 * (delta_R + result_delta_R) * dt;
    F.block<3, 3>(6, 12) = 0.5 * result_delta_R * acc1_skew * dt2;
    F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15,18);
    V.block<3, 3>(0, 6) = 0.5 *Eigen::Matrix3d::Identity() * dt;
    V.block<3, 3>(0, 9) = 0.5 *Eigen::Matrix3d::Identity() * dt;

    V.block<3, 3>(3, 0) = 0.25 * delta_R * dt2;
    V.block<3, 3>(3, 3) =  0.25 * delta_R * dt2;
    V.block<3, 3>(3, 6) =  - 0.125* result_delta_R * acc1_skew  * dt3 ;
    V.block<3, 3>(3, 9) =  - 0.125* result_delta_R * acc1_skew  * dt3 ; 

    V.block<3, 3>(6, 0) =  0.5 * delta_R * dt;
    V.block<3, 3>(6, 3) =  0.5 * result_delta_R * dt;
    V.block<3, 3>(6, 6) = -0.25 * result_delta_R * acc1_skew  * dt2;
    V.block<3, 3>(6, 9) = -0.25 * result_delta_R * acc1_skew  * dt2;
    
    V.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * dt;
    V.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * dt;

    jacobian = F * jacobian;
    covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    
    last_acc = acc;
    last_gyr = gyr;
    sum_dt+=dt;
}

void ImuPreintegrationClass::update(const Eigen::Vector3d& b_a_in, const Eigen::Vector3d& b_g_in){
    b_a = b_a_in;
    b_g = b_g_in;
    initWithImu(acc_buf[0], gyr_buf[0]);
    for(int i=0; i<acc_buf.size(); i++){
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);  
    } 
}

std::vector<Eigen::Vector3d> ImuPreintegrationClass::getAcc(void){
    return acc_buf;
}

std::vector<Eigen::Vector3d> ImuPreintegrationClass::getGyr(void){
    return gyr_buf;
}