#pragma once

#include <iostream>
#include <vtkSTLReader.h>
#include <vtkContourFilter.h>
#include <vtkSurfaceReconstructionFilter.h>

#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

#include <pcl/io/ascii_io.h>

using namespace std;


//读取双脚的stl文件路径,输出切割完左右脚stl文件
bool splitfoots(string StlPath);

bool splitfoots2(string StlPath);