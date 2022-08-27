#include "process_stl.h"
#include <iostream>
#include <io.h>


#include <pcl/io/ply_io.h>


using namespace std;

//��ȡ˫�ŵ�stl�ļ�·��,����и������ҽ�stl�ļ�
bool splitfoots(string StlPath)
{
	cout << "begin splitfoots" << endl;


	
	////����һ
	////��ȡCADģ��
	//vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	//reader->SetFileName(StlPath.c_str());
	//reader->Update();
	////��ת����ply��ʽ
	//vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	//polyData = reader->GetOutput();
	//polyData->GetNumberOfPoints();


	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	////��plyתpcd
	//pcl::io::vtkPolyDataToPointCloud(polyData, *cloud);

	//cout << "total size: " << cloud->points.size() << endl;



	//������
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
	//Y��
	transform_1(0, 0) = cos(theta);
	transform_1(0, 2) = -sin(theta);
	transform_1(2, 0) = sin(theta);
	transform_1(2, 2) = cos(theta);

#else
	float theta = M_PI / 2; // The angle of rotation in radians
	float theta2 = -(M_PI / 2); // The angle of rotation in radians
	//X��
	transform_1(1, 1) = cos(theta);
	transform_1(1, 2) = sin(theta);
	transform_1(2, 1) = -sin(theta);
	transform_1(2, 2) = cos(theta);
#endif



	//Z��
	transform_2(0, 0) = cos(theta2);
	transform_2(0, 1) = sin(theta2);
	transform_2(1, 0) = -sin(theta2);
	transform_2(1, 1) = cos(theta2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// ����ʹ�� transform_1 �� transform_2; t������һ����
	pcl::transformPointCloud(*cloud, *t_cloud, transform_1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	// ����ʹ�� transform_1 �� transform_2; t������һ����
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


	// ����kd-tree������������ .
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(t_cloud2);

	// Euclidean �������.
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;//��EuclideanClusterExtraction�ǻ���ŷʽ������о���ָ����
	clustering.setClusterTolerance(10);//������ŷ�Ͽռ�����ʹ�õ������뾶���õĹ�С���ܵ��¾��౻���ֵ�������Ⱥ�����õĹ�����ܽ����������ͨ
	clustering.setMinClusterSize(2000);// ���þ�������ĵ���С����Ŀ
	clustering.setMaxClusterSize(20000); //���þ�������ĵ�������Ŀ
	clustering.setSearchMethod(kdtree);//��Ĺؼ���Ա����
	clustering.setInputCloud(t_cloud2);//ָ������ĵ��ƽ��о���ָ�
	std::vector<pcl::PointIndices> clusters;// cluster�洢���ƾ���ָ�Ľ����PointIndices�洢��Ӧ�㼯������
	clustering.extract(clusters);


	if (clusters.size() < 2)
		return false;

	// ����ÿһ����
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		//������еĵ��Ƶ�һ���µĵ�����
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		double mean;	//���ƾ�ֵ
		double stddev;	//���Ʊ�׼��
		vector<float> vec_y;
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
		{
			vec_y.push_back(t_cloud2->points[*point].y);
			cluster->points.push_back(t_cloud2->points[*point]);
		}

		pcl::getMeanStd(vec_y, mean, stddev);
		//cout << "\n->����Y����ľ�ֵΪ��" << endl;
		cout << "size: " << cluster->points.size() << " mean: " << mean << endl;

		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ����
		if (cluster->points.size() <= 0)
			continue;

		vtkSmartPointer<vtkPolyData> polyData2 = vtkSmartPointer<vtkPolyData>::New();
		pcl::io::pointCloudTovtkPolyData(*cluster, polyData2);

		vtkSmartPointer<vtkPolyData> points =
			vtkSmartPointer<vtkPolyData>::New();
		points->SetPoints(polyData2->GetPoints()); //�������ģ���еļ������ݣ��㼯


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

//��ȡ˫�ŵ�stl�ļ�·��,����и������ҽ�stl�ļ�
bool splitfoots2(string StlPath)
{
	cout << "begin splitfoots" << endl;

	pcl::PolygonMesh mesh, meshL, meshR;
	pcl::io::loadPolygonFile(StlPath + "/foot.stl", mesh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	// ����ϵУ׼
	Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();

#if 0
	float theta = M_PI; // The angle of rotation in radians
	float theta2 = M_PI / 2; // The angle of rotation in radians
	//Y��
	transform_1(0, 0) = cos(theta);
	transform_1(0, 2) = -sin(theta);
	transform_1(2, 0) = sin(theta);
	transform_1(2, 2) = cos(theta);

#else
	float theta = M_PI / 2; // The angle of rotation in radians
	float theta2 = -(M_PI / 2); // The angle of rotation in radians
	//X��
	transform_y(1, 1) = cos(theta);
	transform_y(1, 2) = sin(theta);
	transform_y(2, 1) = -sin(theta);
	transform_y(2, 2) = cos(theta);
#endif



	//Z��
	transform_z(0, 0) = cos(theta2);
	transform_z(0, 1) = sin(theta2);
	transform_z(1, 0) = -sin(theta2);
	transform_z(1, 1) = cos(theta2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans_y(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::transformPointCloud(*cloud, *cloud_trans_y, transform_y);
	pcl::transformPointCloud(*cloud_trans_y, *cloud_trans, transform_z);



	// ɸѡmesh��

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