#include "process_stl.h"
#include <iostream>

using namespace std;

//��ȡ˫�ŵ�stl�ļ�·��,����и������ҽ�stl�ļ�
bool splitfoots(string StlPath)
{
	cout << "begin splitfoots" << endl;
	//��ȡCADģ��
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(StlPath.c_str());
	reader->Update();
	//��ת����ply��ʽ
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData = reader->GetOutput();
	polyData->GetNumberOfPoints();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//��plyתpcd
	pcl::io::vtkPolyDataToPointCloud(polyData, *cloud);

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

	float theta = M_PI; // The angle of rotation in radians
	float theta2 = M_PI / 2; // The angle of rotation in radians

	//Y��
	transform_1(0, 0) = cos(theta);
	transform_1(0, 2) = -sin(theta);
	transform_1(2, 0) = sin(theta);
	transform_1(2, 2) = cos(theta);

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
		cout << mean << endl;

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