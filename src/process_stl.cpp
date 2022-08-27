#include "process_stl.h"
#include <iostream>
#include <io.h>


#include <pcl/io/ply_io.h>


using namespace std;

//读取双脚的stl文件路径,输出切割完左右脚stl文件
bool splitfoots(string StlPath)
{
	cout << "begin splitfoots" << endl;


	
	////方法一
	////读取CAD模型
	//vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	//reader->SetFileName(StlPath.c_str());
	//reader->Update();
	////先转出到ply格式
	//vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	//polyData = reader->GetOutput();
	//polyData->GetNumberOfPoints();


	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	////从ply转pcd
	//pcl::io::vtkPolyDataToPointCloud(polyData, *cloud);

	//cout << "total size: " << cloud->points.size() << endl;



	//方法二
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFile(StlPath+"/foot.stl", mesh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);


	cout << "total size: " << cloud->points.size() << endl;



	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

#if 0
	float theta = M_PI; // The angle of rotation in radians
	float theta2 = M_PI / 2; // The angle of rotation in radians
	//Y轴
	transform_1(0, 0) = cos(theta);
	transform_1(0, 2) = -sin(theta);
	transform_1(2, 0) = sin(theta);
	transform_1(2, 2) = cos(theta);

#else
	float theta = M_PI / 2; // The angle of rotation in radians
	float theta2 = -(M_PI / 2); // The angle of rotation in radians
	//X轴
	transform_1(1, 1) = cos(theta);
	transform_1(1, 2) = sin(theta);
	transform_1(2, 1) = -sin(theta);
	transform_1(2, 2) = cos(theta);
#endif



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

#if 0
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(t_cloud2, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem();
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif


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
		cout << "size: " << cluster->points.size() << " mean: " << mean << endl;

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
		surf->SetNeighborhoodSize(20);
		surf->SetSampleSpacing(2);
		surf->Update();


		vtkSmartPointer<vtkContourFilter> contourFilter = vtkSmartPointer<vtkContourFilter>::New();
		contourFilter->SetInputConnection(surf->GetOutputPort());
		contourFilter->SetValue(0, 0.0);
		contourFilter->Update();


		////polyData2.
		pcl::PolygonMesh ret;
		pcl::io::vtk2mesh(contourFilter->GetOutput(), ret);

		if (mean > 0)
			pcl::io::savePolygonFileSTL(StlPath + "/L.stl", ret, true);
		else
			pcl::io::savePolygonFileSTL(StlPath + "/R.stl", ret, true);

		currentClusterNum++;
	}

	if (currentClusterNum != 3)
		return false;

	cout << "splitfoots finished" << endl;
	return true;
}

//读取双脚的stl文件路径,输出切割完左右脚stl文件
bool splitfoots2(string StlPath)
{
	cout << "begin splitfoots" << endl;

	pcl::PolygonMesh mesh, meshL, meshR;
	pcl::io::loadPolygonFile(StlPath + "/foot.stl", mesh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	// 坐标系校准
	Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();

#if 0
	float theta = M_PI; // The angle of rotation in radians
	float theta2 = M_PI / 2; // The angle of rotation in radians
	//Y轴
	transform_1(0, 0) = cos(theta);
	transform_1(0, 2) = -sin(theta);
	transform_1(2, 0) = sin(theta);
	transform_1(2, 2) = cos(theta);

#else
	float theta = M_PI / 2; // The angle of rotation in radians
	float theta2 = -(M_PI / 2); // The angle of rotation in radians
	//X轴
	transform_y(1, 1) = cos(theta);
	transform_y(1, 2) = sin(theta);
	transform_y(2, 1) = -sin(theta);
	transform_y(2, 2) = cos(theta);
#endif



	//Z轴
	transform_z(0, 0) = cos(theta2);
	transform_z(0, 1) = sin(theta2);
	transform_z(1, 0) = -sin(theta2);
	transform_z(1, 1) = cos(theta2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans_y(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::transformPointCloud(*cloud, *cloud_trans_y, transform_y);
	pcl::transformPointCloud(*cloud_trans_y, *cloud_trans, transform_z);



	// 筛选mesh面

	meshL.polygons.resize(0);
	meshR.polygons.resize(0);
	std::cout << "All Polygons" << std::endl;
	std::cout << mesh.polygons.size() << std::endl;

	std::vector<pcl::Vertices> polysL;
	std::vector<pcl::Vertices> polysR;
	for (int i = 0; i < mesh.polygons.size(); i++)
	{
		int idx_point1 = mesh.polygons[i].vertices[0];
		int idx_point2 = mesh.polygons[i].vertices[1];
		int idx_point3 = mesh.polygons[i].vertices[2];

		float y_point1 = cloud_trans->points[idx_point1].y;
		float y_point2 = cloud_trans->points[idx_point2].y;
		float y_point3 = cloud_trans->points[idx_point3].y;

		pcl::Vertices vertice;

		if (y_point1 > 0 && y_point2 > 0 && y_point3 > 0)
		{
			//pcl::Vertices v1;
			vertice.vertices.push_back(mesh.polygons[i].vertices[0]);
			vertice.vertices.push_back(mesh.polygons[i].vertices[1]);
			vertice.vertices.push_back(mesh.polygons[i].vertices[2]);
			polysL.push_back(vertice);
		}
		else if (y_point1 < 0 && y_point2 < 0 && y_point3 < 0)
		{
			//pcl::Vertices v2;
			vertice.vertices.push_back(mesh.polygons[i].vertices[0]);
			vertice.vertices.push_back(mesh.polygons[i].vertices[1]);
			vertice.vertices.push_back(mesh.polygons[i].vertices[2]);
			polysR.push_back(vertice);
		}


	}
	meshL.polygons = polysL;
	meshL.cloud.data.clear();
	meshL.cloud.width = meshL.cloud.height = 0;
	meshL.cloud.is_dense = true;

	meshR.polygons = polysR;
	meshR.cloud.data.clear();
	meshR.cloud.width = meshR.cloud.height = 0;
	meshR.cloud.is_dense = true;


	pcl::toPCLPointCloud2(*cloud_trans, meshL.cloud);
	pcl::toPCLPointCloud2(*cloud_trans, meshR.cloud);
	pcl::io::savePolygonFileSTL(StlPath + "/L.stl", meshL, true);
	pcl::io::savePolygonFileSTL(StlPath + "/R.stl", meshR, true);

	cout << "splitfoots finished" << endl;
	return true;
}




int main(){
	//string stlpath = "D:/work/completed/measurement/zbbdata/model_12355678999/G6BE93400KG9-20220823-141051/3DModel";
	string stlpath = "D:/work/completed/measurement/zbbdata/0825_analyst/model2_12345678987/G6BE93400KG9-20220823-142443/3DModel";

	splitfoots2(stlpath);

    return 0;
} 