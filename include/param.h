#ifndef _PARAM_H_
#define _PARAM_H_

// opencv
#include <opencv2/opencv.hpp>

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
// define lidar parameter
class CommonParam{
    public:
        CommonParam(){};
        void loadParam(std::string& path);
        int getCoreNum(void);
        int getInitFrame(void);
        int getNearbyFrame(void);
        Eigen::Isometry3d getTbl(void);
    private:
        int core_num;
        int init_frame;
        int nearby_frame;
        Eigen::Isometry3d Tbl;
};

// define lidar parameter
class LidarParam{
    public:
        LidarParam(){};
        void loadParam(std::string& path);
        int getFrequency(void);
        int getScanLine(void);
        double getScanPeroid(void);
        double getMinDistance(void);
        double getMaxDistance(void);
        double getVerticalAngle(void);
        double getMapResolution(void);
        Eigen::Matrix<double, 6, 1> getOdomN(void);
        double getEdgeN();
        double getSurfN();
        double getPlanmotion_rotx();
        double getPlanmotion_roty();
        double getPlanmotion_z();
        double getSigma_uscale();
        double getLocalMapSize(void);
        double getLocalMapResolution(void);
        double getMapCellWidth(void);
        double getMapCellHeight(void);
        double getMapCellDepth(void);
        int getMapCellWidthRange(void);
        int getMapCellHeightRange(void);
        int getMapCellDepthRange(void);
    private:
        int frequency;
        int scan_line;
        double scan_period;
        double min_distance;
        double max_distance;
        double vertical_angle;
        double map_resolution;
        Eigen::Matrix<double, 6, 1> odom_n;
        double edge_n;
        double surf_n;
        double planmotion_rotx;
        double planmotion_roty;
        double planmotion_z;
        double sigma_uscale;
        double local_map_resolution;
        double local_map_size;
        double map_cell_width;
        double map_cell_height;
        double map_cell_depth;
        int map_cell_width_range;
        int map_cell_height_range;
        int map_cell_depth_range;
};

class ImuParam{
    public:
        ImuParam(){};
        void loadParam(std::string& path);
        int getFrequency(void);
        double getAccN(void);
        double getGyrN(void);
        double getAccW(void);
        double getGyrW(void);
    private:
        int frequency;
        double acc_n;
        double gyr_n;
        double acc_w;
        double gyr_w;
};


#endif // _PARAM_H_

