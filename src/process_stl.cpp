#include "process_stl.h"
#include <iostream>

using namespace std;

//读取双脚的stl文件路径,输出切割完左右脚stl文件
bool splitfoots(string StlPath)
{
	cout << "begin splitfoots" << endl;
	//读取CAD模型
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(StlPath.c_str());
	reader->Update();
	//先转出到ply格式
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData = reader->GetOutput();
	polyData->GetNumberOfPoints();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//从ply转pcd
	pcl::io::vtkPolyDataToPointCloud(polyData, *cloud);

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

	float theta = M_PI; // The angle of rotation in radians
	float theta2 = M_PI / 2; // The angle of rotation in radians

	//Y轴
	transform_1(0, 0) = cos(theta);
	transform_1(0, 2) = -sin(theta);
	transform_1(2, 0) = sin(theta);
	transform_1(2, 2) = cos(theta);

	//Z轴
	transform_2(0, 0) = cos(theta2);
	transform_2(0, 1) = sin(theta2);
	transform_2(1, 0) = -sin(theta2);
	transform_2(1, 1) = cos(theta2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// 可以使用 transform_1 或 transform_2; t它们是一样的
	pcl::transformPointCloud(*cloud, *t_cloud, transform_1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	// 可以使用 transform_1 或 transform_2; t它们是一样的
	pcl::transformPointCloud(*t_cloud, *t_cloud2, transform_2);


	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZ>(t_cloud2, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//viewer->addCoordinateSystem();
	//viewer->initCameraParameters();

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	//


	// 建立kd-tree对象用来搜索 .
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(t_cloud2);

	// Euclidean 聚类对象.
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;//类EuclideanClusterExtraction是基于欧式距离进行聚类分割的类
	clustering.setClusterTolerance(10);//设置在欧氏空间中所使用的搜索半径设置的过小可能导致聚类被划分到几个集群，设置的过大可能将聚类进行联通
	clustering.setMinClusterSize(2000);// 设置聚类包含的的最小点数目
	clustering.setMaxClusterSize(20000); //设置聚类包含的的最大点数目
	clustering.setSearchMethod(kdtree);//类的关键成员函数
	clustering.setInputCloud(t_cloud2);//指定输入的点云进行聚类分割
	std::vector<pcl::PointIndices> clusters;// cluster存储点云聚类分割的结果。PointIndices存储对应点集的索引
	clustering.extract(clusters);


	if (clusters.size() < 2)
		return false;

	// 遍历每一点云
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		//添加所有的点云到一个新的点云中
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		double mean;	//点云均值
		double stddev;	//点云标准差
		vector<float> vec_y;
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
		{
			vec_y.push_back(t_cloud2->points[*point].y);
			cluster->points.push_back(t_cloud2->points[*point]);
		}

		pcl::getMeanStd(vec_y, mean, stddev);
		//cout << "\n->点云Y坐标的均值为：" << endl;
		cout << mean << endl;

		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// 保存
		if (cluster->points.size() <= 0)
			continue;

		vtkSmartPointer<vtkPolyData> polyData2 = vtkSmartPointer<vtkPolyData>::New();
		pcl::io::pointCloudTovtkPolyData(*cluster, polyData2);

		vtkSmartPointer<vtkPolyData> points =
			vtkSmartPointer<vtkPolyData>::New();
		points->SetPoints(polyData2->GetPoints()); //获得网格模型中的几何数据：点集


		vtkSmartPointer<vtkSurfaceReconstructionFilter> surf =
			vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
		surf->SetInputData(points);
		surf->Update();


		vtkSmartPointer<vtkContourFilter> contourFilter = vtkSmartPointer<vtkContourFilter>::New();
		contourFilter->SetInputConnection(surf->GetOutputPort());
		contourFilter->SetValue(0, 0.0);
		contourFilter->Update();


		////polyData2.
		pcl::PolygonMesh ret;
		pcl::io::vtk2mesh(contourFilter->GetOutput(), ret);

		if (mean > 0)
			pcl::io::savePolygonFileSTL("L.stl", ret, true);
		else
			pcl::io::savePolygonFileSTL("R.stl", ret, true);

		currentClusterNum++;
	}

	if (currentClusterNum != 3)
		return false;

	cout << "splitfoots finished" << endl;
	return true;
}


int main(){
  
    cout << "Enter the dir path of stl model:" << endl;
	string stlpath = "foot.stl";
	bool ret = splitfoots(stlpath);
	cout << boolalpha << ret << endl;
    return 0;
} 