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
#include <pcl/surface/mls.h> //��С���� �ز���ƽ��
#include <pcl/surface/poisson.h>  //�����ؽ�
#include <pcl/geometry/polygon_mesh.h> //MESH
#include <pcl/surface/gp3.h>  //̰��������


#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <opencv2/core.hpp>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkContourFilter.h>
#include <pcl/common/transforms.h>
//#include <pcl/point_cloud.h>


using namespace std;


//��ȡ˫�ŵ�asc�ļ�,ת����pcl::PointXYZ���Ʊ���
void readasc(string file_name, pcl::PointCloud<pcl::PointXYZ>& cloud);

//��ȡ˫�ŵ�stl�ļ�·��,����и������ҽ�stl�ļ�
void splitfoots(string StlPath)