//
// Created by czy on 2022/3/19.
//

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

Eigen::Matrix4f read_T(std::string &Rtpath) {
    std::stringstream T_stream(Rtpath);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    std::string s;
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            std::getline(T_stream, s, ' ');
            T(i, j) = stof(s);
        }
    }
    return T;
}

// ICP配准 返回变换矩阵
Eigen::Matrix4f registration_icp(const pcl::PointCloud<pcl::PointXYZ> &src_, const pcl::PointCloud<pcl::PointXYZ> &tgt_) {
    Eigen::Matrix4f matrix ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>(src_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>(tgt_));
    pcl::PointCloud<pcl::PointXYZ> out_cloud;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setTransformationEpsilon(1e-10);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setMaximumIterations(200);
    icp.setUseReciprocalCorrespondences(true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*icp_cloud);
    matrix = icp.getFinalTransformation();
//    std::cout << matrix.matrix() << std::endl;
    return matrix;
}

// 读取点云
void read_pcd_data(const std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ> &point_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcd_path, *cloud);
    pcl::VoxelGrid<pcl::PointXYZ> s;
    s.setInputCloud(cloud);
    s.setLeafSize(0.005f, 0.005f, 0.005f);
    s.filter(point_cloud);
}

// 计算重叠率 thre_dis 表示为距离的阈值，代码中介绍
float cal_overlap_ratio(const pcl::PointCloud<pcl::PointXYZ> &src_, const pcl::PointCloud<pcl::PointXYZ> &tgt_,
                        float thre_dis) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>(src_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>(tgt_));

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(tgt);
    int overlap_point_num = 0;
    float overlap_ratio;

    std::vector<int> search_indices; // 查询点的索引
    std::vector<float> distances_square;   // 保存点之间的距离

    int down_rate = 2;

    for (int i = 0; i < src->points.size(); i++) {  // 对src中所有点进行遍历
        if (i % down_rate == 0) {  // 考虑到点数量问题，间隔行取点
            tree.nearestKSearch(src->points[i], 1, search_indices, distances_square);  // 查找最近点，保存索引和距离
            if (distances_square[0] < thre_dis * thre_dis)  // 查询的距离和阈值进行对比，如果足够小则为重叠点，否则，弃之
                overlap_point_num++;
            std::vector<int>().swap(search_indices); // 清空
            std::vector<float>().swap(distances_square);
        }
    }
    // 重叠点/总点 = 重叠率，总点受间隔距离影响，所以/间隔
    overlap_ratio = (0.001 + overlap_point_num) / (src->points.size() / down_rate + down_rate - 1);

    return overlap_ratio;
}
// 随机移动点云 测试配准，仅测试用
Eigen::Matrix4f random_rotate(pcl::PointCloud<pcl::PointXYZ> &src) {
//    Eigen::Matrix3f R = Eigen::Matrix3f::Random();
//    Eigen::Matrix<float, 3,1> t = Eigen::Matrix<float, 3,1>::Random();
    Eigen::Matrix<float, 3, 1> t;
    t << 0.5, 0, 0;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
//    T.topLeftCorner<3,3>() = R;
    T.topRightCorner<3, 1>() = t;
    std::cout << "random T: \n" << T << std::endl;
    pcl::transformPointCloud(src, src, T);

    return T;
}

void show(const pcl::PointCloud<pcl::PointXYZ> &src_, const pcl::PointCloud<pcl::PointXYZ> &tgt_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>(src_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>(tgt_));

    // vis ICP registration
    pcl::visualization::PCLVisualizer viewer("ICP配准结果");
    viewer.setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(src, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(src, "target cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    // 对源点云着色可视化 (blue).
    // 对转换后的源点云着色 (green)可视化.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_color(tgt, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(tgt, icp_color, "icp cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "icp cloud");
    // 启动可视化
    //viewer->addCoordinateSystem(0.1);  //显示XYZ指示轴
    //viewer->initCameraParameters();   //初始化摄像头参数
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ratio");
    ros::NodeHandle n("~");
    std::string src_path, tgt_path, Rt_path;
    bool has_Rt;
    // 读取launch中参数
    n.getParam("src_path", src_path);
    n.getParam("tgt_path", tgt_path);
    n.getParam("tgt_path", Rt_path);
    n.getParam("tgt_path", has_Rt);
    std::cout << "src_path: " << src_path << std::endl;
    std::cout << "tgt_path: " << tgt_path << std::endl;

    pcl::PointCloud<pcl::PointXYZ> src_point, tgt_point;
    read_pcd_data(src_path, src_point); // 读取点云
    read_pcd_data(tgt_path, tgt_point);

    std::cout << "src_point.size(): " << src_point.size() << std::endl;
    std::cout << "tgt_point.size(): " << tgt_point.size() << std::endl;

    Eigen::Matrix4f T;
    float OverlapRatio;
    pcl::PointCloud<pcl::PointXYZ> src_T;
    if (has_Rt) {
        T = read_T(Rt_path);  // 读取变换矩阵，如果有的话
        pcl::transformPointCloud(src_point, src_T, T);
        OverlapRatio = cal_overlap_ratio(src_T, tgt_point, 0.1);
    } else {
        // test random rotate point cloud
//        T = random_rotate(src_point);
        std::cout << "Will Registration src and tgt by ICP algorithm" << std::endl;
        T = registration_icp(src_point, tgt_point);
        std::cout << "T: \n" << T << std::endl;
        pcl::transformPointCloud(src_point, src_T, T);
        show(src_T, tgt_point);
        OverlapRatio = cal_overlap_ratio(src_T, tgt_point, 0.01);

    }

    std::cout << "OverlapRatio: \t" << OverlapRatio << std::endl;
    std::cout << "Done \n";

    return 0;
}
