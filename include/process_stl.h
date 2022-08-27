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


//��ȡ˫�ŵ�stl�ļ�·��,����и������ҽ�stl�ļ�
bool splitfoots(string StlPath);

bool splitfoots2(string StlPath);