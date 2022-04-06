#pragma once

#include <iostream>
#include <vtkSTLReader.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h> //最小二乘 重采样平滑
#include <pcl/surface/poisson.h>  //泊松重建
#include <pcl/geometry/polygon_mesh.h> //MESH
#include <pcl/surface/gp3.h>  //贪心三角形


#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <opencv2/core.hpp>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkContourFilter.h>
#include <pcl/common/transforms.h>
//#include <pcl/point_cloud.h>


using namespace std;


//读取双脚的asc文件,转换成pcl::PointXYZ点云变量
void readasc(string file_name, pcl::PointCloud<pcl::PointXYZ>& cloud);

//读取双脚的stl文件路径,输出切割完左右脚stl文件
void splitfoots(string StlPath)