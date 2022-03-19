//
// Created by czy on 2022/3/19.
//
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
using namespace std;

void show(const pcl::PointCloud<pcl::PointXYZ> &src_, const pcl::PointCloud<pcl::PointXYZ> &tgt_){
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>(src_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>(tgt_));

    // vis ICP registration
    pcl::visualization::PCLVisualizer viewer("ICP配准结果");
    viewer.setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(src, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(src,target_color, "target cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    // 对转换后的源点云着色 (green)可视化.
    Eigen::Matrix4f T = Eigen::Matrix4f ::Identity();
    Eigen::Matrix<float, 3,1> t ;
    t << 0.001,0,0;
    T.topRightCorner<3,1>() = t;
    pcl::transformPointCloud(*tgt,*tgt, T);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>icp_color(tgt, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(tgt, icp_color, "icp cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "icp cloud");
    // 启动可视化
    //viewer->addCoordinateSystem(0.1);  //显示XYZ指示轴
    //viewer->initCameraParameters();   //初始化摄像头参数
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "generate");
    ros::NodeHandle n("~");

//    std::string pcd_path (argv[1]);
    std::string  pcd_path;
    n.getParam("pcd_path", pcd_path);
    std::string save_path;
    n.getParam("save_path", save_path);
//    float ratio = stof(argv[2]);
    double  ratio;  // 重叠率
    n.getParam("ratio", ratio);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcd_path, *cloud);  // 读取点云
    unsigned long nums = cloud->size();
    unsigned long filterNum = nums * ratio;  // 计算重叠的点
    // 在(filterNum / 2)------ (nums - filterNum / 2)范围内生成一个随机数，在该随机数左右取足够的点
    unsigned long r = rand()%((nums - filterNum / 2) - (filterNum / 2) + 1) + (filterNum / 2);
    std::vector<int> index(filterNum);
//    生成filterNum大小的序列，
//    考虑到点云的索引和随机数，所以左右两各减去filterNum/2
    unsigned long start = r - filterNum/2;
    unsigned long end = r + filterNum/2;
    for (unsigned long i = start; i < end; ++i) {
        index[i - start] = i;
    }
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices(index_ptr);
    extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    extract.filter (*cloud_p);
    pcl::io::savePCDFile(save_path, *cloud_p);
    cout << "save success" << endl;
    show(*cloud, *cloud_p);




    return 0;
}