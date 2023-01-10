
#ifndef _LIDAR_OPTIMIZATION_FACTOR_H_
#define _LIDAR_OPTIMIZATION_FACTOR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils.h"
#include <ros/ros.h>
class LidarOdometryFactor : public ceres::SizedCostFunction<6, 15, 15>{
public:
    LidarOdometryFactor(Eigen::Isometry3d odom_in, Eigen::Matrix<double, 6, 1> covariance_in);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    Eigen::Isometry3d odom;
    Eigen::Matrix<double, 6, 6> sqrt_info;
};

class LidarEdgeFactor : public ceres::SizedCostFunction<1, 15> { //3,15
public:
	LidarEdgeFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d last_point_a_in, Eigen::Vector3d last_point_b_in, double covariance_in,double planmotion_rotx,  double planmotion_roty, double planmotion_z, double scale );
	virtual ~LidarEdgeFactor() {}
	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

	Eigen::Vector3d curr_point;
	Eigen::Vector3d last_point_a;
	Eigen::Vector3d last_point_b;
	double sqrt_info;
	double noisex;
   	double noisey;
    double noisez;
    double sigma_scale;

};

class LidarSurfFactor : public ceres::SizedCostFunction<1, 15> { //3,15
public:
	LidarSurfFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d plane_unit_norm_in, double negative_OA_dot_norm_in, double covariance_in, double planmotion_rotx,  double planmotion_roty, double planmotion_z, double scale );
	virtual ~LidarSurfFactor() {}
	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
	double sqrt_info;
	double noisex;
   	double noisey;
    double noisez;
    double sigma_scale;
};


#endif // _LIDAR_OPTIMIZATION_FACTOR_H_

