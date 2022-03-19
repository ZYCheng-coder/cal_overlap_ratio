//
// Created by czy on 2022/3/19.
//

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv) {


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>());

    // load from ply
    pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *cloud_source);
    pcl::io::loadPLYFile<pcl::PointXYZ>(argv[2], *cloud_target);
    // save as pcd
    pcl::io::savePCDFile("src_full.pcd", *cloud_source);
    pcl::io::savePCDFile("tgt_full.pcd", *cloud_target);


    return (0);
}
