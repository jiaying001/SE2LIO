#include "param.h"

std::string readString(cv::FileNode& file_node, const std::string& name, const std::string& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value"<< default_value << std::endl;
    else if(!node.isString())
        std::cerr << name << " parameter must be a string, using default value"<< default_value << std::endl;
    else
        return node.string();
    return default_value;
}
int readInt(cv::FileNode& file_node, const std::string& name, const int& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value"<< default_value << std::endl;
    else if(!node.isInt())
        std::cerr << name << " parameter must be an integer, using default value"<< default_value << std::endl;
    else
        return node.operator int();
    return default_value;
}
double readDouble(cv::FileNode& file_node, const std::string& name, const double& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value"<< default_value << std::endl;
    else if(!node.isReal())
        std::cerr << name << " parameter must be a double, using default value"<< default_value << std::endl;
    else
        return node.real();
    return default_value;
}
Eigen::Isometry3d readMatrix(cv::FileNode& file_node, const std::string& name, const Eigen::Isometry3d& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value: rotation["<< default_value.linear() <<"] translation"<< default_value.translation()<< std::endl;
    else{
        cv::Mat mat_in = node.mat();
        if(mat_in.empty() || mat_in.rows != 4 || mat_in.cols != 4)
            std::cerr << name << " required parameter does not exist, using default value: rotation["<< default_value.linear() <<"] translation"<< default_value.translation()<< std::endl;
        else{
            Eigen::Isometry3d mat_out = Eigen::Isometry3d::Identity();
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    mat_out.linear()(i,j) = mat_in.at<double>(i, j);
                }
            }
            mat_out.translation().x() = mat_in.at<double>(0, 3);
            mat_out.translation().y() = mat_in.at<double>(1, 3);
            mat_out.translation().z() = mat_in.at<double>(2, 3);
            return mat_out; 
        }
    }
    return default_value;
}

void CommonParam::loadParam(std::string& path){
    //init qr detection 
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;
    cv::FileNode node = fsSettings["common"]; 
    core_num = readInt(node,"core_num",1);
    init_frame = readInt(node,"init_frame",20);
    nearby_frame = readInt(node,"nearby_frame",15);
    Tbl = readMatrix(node,"Tbl",Eigen::Isometry3d::Identity());
    fsSettings.release();
}
int CommonParam::getCoreNum(){
    return core_num;
}
int CommonParam::getInitFrame(){
    return init_frame;
}
int CommonParam::getNearbyFrame(){
    return nearby_frame;
}
Eigen::Isometry3d CommonParam::getTbl(){
    return Tbl;
}

void LidarParam::loadParam(std::string& path){
    //init qr detection 
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;
    cv::FileNode node = fsSettings["lidar"];
    scan_line = readInt(node,"scan_line",64);
    scan_period = readDouble(node,"scan_period",0.1);
    min_distance = readDouble(node,"min_distance",3.0);
    max_distance = readDouble(node,"max_distance",90.0);
    vertical_angle = readDouble(node,"vertical_angle",2.0);
    map_resolution = readDouble(node,"map_resolution",0.05);
    frequency = readInt(node,"frequency",30);

   double odom_n_rpy = readDouble(node,"odom_n_rpy",1e-3);
   double odom_n_xyz = readDouble(node,"odom_n_xyz",1e-3);
   for (int i = 0; i < 3; ++i){
       odom_n(i) = odom_n_rpy;
       odom_n(i+3) = odom_n_xyz;
   }
    edge_n = readDouble(node,"edge_n",1e-3);
    surf_n = readDouble(node,"surf_n",1e-3);
    local_map_size = readDouble(node,"local_map_size",10.0);
    local_map_resolution = readDouble(node,"local_map_resolution",0.05);

    planmotion_rotx = readDouble(node,"planmotion_rotx",1e-3);
    planmotion_roty = readDouble(node,"planmotion_roty",1e-3);
    planmotion_z = readDouble(node,"planmotion_z",1e-3);
    sigma_uscale = readDouble(node,"sigma_uscale",1);

    map_cell_width = readDouble(node,"map_cell_width",50.0);
    map_cell_height = readDouble(node,"map_cell_height",50.0);
    map_cell_depth = readDouble(node,"map_cell_depth",50.0);
    map_cell_width_range = readInt(node,"map_cell_width_range",2);
    map_cell_height_range = readInt(node,"map_cell_height_range",2);
    map_cell_depth_range = readInt(node,"map_cell_depth_range",2);
    fsSettings.release();
}
int LidarParam::getFrequency(){
    return frequency;
}
int LidarParam::getScanLine(){
    return scan_line;
}
double LidarParam::getScanPeroid(){
    return scan_period;
}
double LidarParam::getMinDistance(){
    return min_distance;
}
double LidarParam::getMaxDistance(){
    return max_distance;
}
double LidarParam::getVerticalAngle(){
    return vertical_angle;
}
double LidarParam::getMapResolution(){
    return map_resolution;
}
Eigen::Matrix<double, 6, 1> LidarParam::getOdomN(){
    return odom_n;
}
double LidarParam::getEdgeN(){
    return edge_n;
}
double LidarParam::getSurfN(){
    return surf_n;
}
double LidarParam::getPlanmotion_rotx(){
    return planmotion_rotx;
}
double LidarParam::getPlanmotion_roty(){
    return planmotion_roty;
}
double LidarParam::getPlanmotion_z(){
    return planmotion_z;
}
double LidarParam::getSigma_uscale(){
    return sigma_uscale;
}
double LidarParam::getLocalMapResolution(){
    return local_map_resolution;
}
double LidarParam::getLocalMapSize(){
    return local_map_size;
}
double LidarParam::getMapCellWidth(){
    return map_cell_width;
}
double LidarParam::getMapCellHeight(){
    return map_cell_height;
}
double LidarParam::getMapCellDepth(){
    return map_cell_depth;
}
int LidarParam::getMapCellWidthRange(){
    return map_cell_width_range;
}
int LidarParam::getMapCellHeightRange(){
    return map_cell_height_range;
}
int LidarParam::getMapCellDepthRange(){
    return map_cell_depth_range;
}

void ImuParam::loadParam(std::string& path){
    //init qr detection 
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;
    cv::FileNode node = fsSettings["imu"]; 
    frequency = readInt(node,"frequency",200);
    double freq_sr = sqrt((double)frequency);
    double acc_n_in = readDouble(node,"acc_n",1e-2);
    double gyr_n_in = readDouble(node,"gyr_n",1e-3);
    double acc_w_in = readDouble(node,"acc_w",1e-3);
    double gyr_w_in = readDouble(node,"gyr_w",1e-4);
    acc_n = acc_n_in * freq_sr;
    gyr_n = gyr_n_in * freq_sr;
    acc_w = acc_w_in / freq_sr;
    gyr_w = gyr_w_in / freq_sr;
    fsSettings.release();
}
int ImuParam::getFrequency(){
    return frequency;
}
double ImuParam::getAccN(){
    return acc_n;
}
double ImuParam::getGyrN(){
    return gyr_n;
}
double ImuParam::getAccW(){
    return acc_w;
}
double ImuParam::getGyrW(){
    return gyr_w;
}
