// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidarOptimizationFactor.h"

LidarOdometryFactor::LidarOdometryFactor(Eigen::Isometry3d odom_in, Eigen::Matrix<double, 6, 1> covariance_in){
    odom = odom_in;
    sqrt_info = covariance_in.asDiagonal().inverse();
}

bool LidarOdometryFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d ri(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Matrix3d Ri = Utils::so3ToR(ri);
    Eigen::Vector3d Pi(parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d rj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Matrix3d Rj = Utils::so3ToR(rj);
    Eigen::Vector3d Pj(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Matrix3d Rij = odom.linear();
    Eigen::Vector3d Pij = odom.translation();

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);

    Eigen::Vector3d so3 = Utils::RToso3(Rij.transpose() * Ri.transpose() * Rj);
    residual.block<3, 1>(0, 0) = so3;
    residual.block<3, 1>(3, 0) = Rij.transpose() * ( Ri.transpose() * (Pj - Pi)- Pij);
    residual = sqrt_info * residual;
    if (jacobians != NULL){  
        if (jacobians[0] != NULL){
            Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_i(jacobians[0]);
            jacobian_i.setZero();
            jacobian_i.block<3, 3>(0, 0) = - Utils::Jr_so3_inv(so3) * Rj.transpose() * Ri;
            jacobian_i.block<3, 3>(3, 0) = Rij.transpose() * Utils::skew( Ri.transpose() * (Pj - Pi));
            jacobian_i.block<3, 3>(3, 3) = - Rij.transpose() * Ri.transpose();
            jacobian_i = sqrt_info * jacobian_i;

            if (jacobian_i.maxCoeff() > 1e8 || jacobian_i.minCoeff() < -1e8)
                ROS_WARN("numerical unstable in odom factor");
        }
        if (jacobians[1] != NULL){
            Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
            jacobian_j.setZero();
            jacobian_j.block<3, 3>(0, 0) = Utils::Jr_so3_inv(so3);
            jacobian_j.block<3, 3>(3, 3) = Rij.transpose() * Ri.transpose();
            jacobian_j = sqrt_info * jacobian_j;

            if (jacobian_j.maxCoeff() > 1e8 || jacobian_j.minCoeff() < -1e8)
                ROS_WARN("numerical unstable in odom factor");
        }
    }
    return true;
}

LidarEdgeFactor::LidarEdgeFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d last_point_a_in, Eigen::Vector3d last_point_b_in, double covariance_in,double planmotion_rotx,  double planmotion_roty, double planmotion_z, double scale){
    curr_point = curr_point_in; 
    last_point_a = last_point_a_in;
    last_point_b = last_point_b_in;
    sqrt_info = 1.0 / covariance_in;
    noisex= planmotion_rotx;
    noisey= planmotion_roty;
    noisez= planmotion_z;
    sigma_scale= scale;
}

bool LidarEdgeFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d ri(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d Pi(parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Matrix3d Ri = Utils::so3ToR(ri);

    Eigen::Vector3d lp = Ri * curr_point + Pi; 
    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    Eigen::Vector3d de = last_point_a - last_point_b;
  
    Eigen::Matrix3d de_byp= -Utils::skew(de)/de.norm();  

    Eigen::Matrix2d Sigma_rotxy;
    Sigma_rotxy<<  noisex,0,0,noisey;
    Eigen::Matrix3d skew_rp =  Ri * Utils::skew(curr_point);
    Eigen::Matrix<double,3,2> J_rotxy = (de_byp*skew_rp).block<3,2>(0,0);
    Eigen::Matrix<double,3,1> J_z;
    J_z= de_byp.block<3,1>(0,2);
    Eigen::Matrix3d Sigma_u = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Sigma_xy = J_rotxy* Sigma_rotxy*J_rotxy.transpose();
    Eigen::Matrix3d Sigma_z= noisez* J_z*J_z.transpose();
    Eigen::Matrix3d cov_matrix = Sigma_xy + Sigma_z+ sigma_scale*Sigma_u;
    Eigen::Matrix3d sqrt_info=Eigen::LLT<Eigen::Matrix3d>(cov_matrix.inverse()).matrixL().transpose();
    nu= sqrt_info * nu;
    residuals[0] = nu.x()/de.norm();
    residuals[1] = nu.y()/de.norm();
    residuals[2] = nu.z()/de.norm();
    
   if(jacobians != NULL){
        if(jacobians[0] != NULL){
            Eigen::Matrix<double, 3, 15> dp_by_so3xyz = Eigen::Matrix<double, 3, 15>::Zero();
            dp_by_so3xyz.block<3, 3>(0, 0) = - Ri * Utils::skew(curr_point);
            dp_by_so3xyz.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
            Eigen::Matrix<double, 15, 15>  info_j;
            info_j.setZero();
            info_j(0,0) = 0;
            info_j(1,1) = 0;
            info_j(2,2) = 1;
            info_j(3,3) = 1;
            info_j(4,4) = 1;
            info_j(5,5) = 0;
            Eigen::Matrix<double, 3, 15> dp_by_so3xyz_new=dp_by_so3xyz*info_j;
            Eigen::Map<Eigen::Matrix<double, 3, 15, Eigen::RowMajor> > jacobian_i(jacobians[0]);
            jacobian_i.setZero();
            jacobian_i = - sqrt_info * Utils::skew(de) * dp_by_so3xyz_new / de.norm();
        }
    }  

    return true;
} 


LidarSurfFactor::LidarSurfFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d plane_unit_norm_in, double negative_OA_dot_norm_in, double covariance_in, double planmotion_rotx,  double planmotion_roty, double planmotion_z, double scale){
    curr_point = curr_point_in;
    plane_unit_norm = plane_unit_norm_in;
    negative_OA_dot_norm = negative_OA_dot_norm_in;
    sqrt_info = 1.0 / covariance_in;
    noisex= planmotion_rotx;
    noisey= planmotion_roty;
    noisez= planmotion_z;
    sigma_scale= scale;
}

bool LidarSurfFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d ri(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d Pi(parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Matrix3d Ri = Utils::so3ToR(ri);
    Eigen::Vector3d lp = Ri * curr_point + Pi;

    Eigen::Vector3d negative_OA=negative_OA_dot_norm*plane_unit_norm;
    Eigen::Matrix3d dh_byp=plane_unit_norm * plane_unit_norm.transpose();
    residuals[0] = sqrt_info * (plane_unit_norm.dot(lp) + negative_OA_dot_norm);

    Eigen::Matrix2d Sigma_rotxy;
    Sigma_rotxy<<  noisex,0,0,noisey;
    Eigen::Matrix3d skew_rp =  Ri * Utils::skew(curr_point);
    Eigen::Matrix<double,3,2> J_rotxy = (dh_byp*skew_rp).block<3,2>(0,0);
    Eigen::Matrix<double,3,1> J_z;
    J_z= dh_byp.block<3,1>(0,2);
    Eigen::Matrix3d Sigma_u = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Sigma_xy = J_rotxy* Sigma_rotxy*J_rotxy.transpose();
    Eigen::Matrix3d Sigma_z= noisez* J_z*J_z.transpose();
    Eigen::Matrix3d cov_matrix =  Sigma_xy + Sigma_z+  sigma_scale*Sigma_u;
    Eigen::Matrix3d sqrt_info2=Eigen::LLT<Eigen::Matrix3d>(cov_matrix.inverse()).matrixL().transpose();
    double dist=plane_unit_norm.dot(lp + negative_OA);
    Eigen::Vector3d dist_v=sqrt_info*dist*plane_unit_norm;

    residuals[0]=dist_v.x();
    residuals[1]=dist_v.y();
    residuals[2]=dist_v.z();

 
    if(jacobians != NULL){
        if(jacobians[0] != NULL){
            Eigen::Matrix<double, 3, 15> dp_by_so3xyz = Eigen::Matrix<double, 3, 15>::Zero();
            dp_by_so3xyz.block<3,3>(0, 0) = - Ri * Utils::skew(curr_point);
            dp_by_so3xyz.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();
            Eigen::Matrix<double, 15, 15> info_j;
            info_j.setZero();
            info_j(0,0) = 0;
            info_j(1,1) = 0;
            info_j(2,2) = 1;
            info_j(3,3) = 1;
            info_j(4,4) = 1;
            info_j(5,5) = 0;
            Eigen::Matrix<double, 3, 15> dp_by_so3xyz_new=dp_by_so3xyz*info_j;
            Eigen::Map<Eigen::Matrix<double, 3, 15, Eigen::RowMajor> > jacobian_i(jacobians[0]);
            jacobian_i.setZero();
            jacobian_i = sqrt_info *plane_unit_norm *plane_unit_norm.transpose() * dp_by_so3xyz_new;
        }
    }
    return true;
}   
