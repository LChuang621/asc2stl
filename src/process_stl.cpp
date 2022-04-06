#include "process_stl.h"



// 读取双脚的asc文件,转换成pcl::PointXYZ点云变量
void readasc(string file_name, pcl::PointCloud<pcl::PointXYZ>& cloud)
{

	// 初始化cloud变量
	std::vector<pcl::PCLPointField> fields;
	pcl::for_each_type<typename pcl::traits::fieldList<pcl::PointXYZ>::type>(pcl::detail::FieldAdder<pcl::PointXYZ>(fields));

	// 检查 XYZ 是否存在
	int x_idx = -1, y_idx = -1, z_idx = -1;
	for (std::size_t d = 0; d < fields.size(); ++d)
	{
		if (fields[d].name == "x")
			x_idx = fields[d].offset;
		else if (fields[d].name == "y")
			y_idx = fields[d].offset;
		else if (fields[d].name == "z")
			z_idx = fields[d].offset;
	}

	// 读入asc文件，把每一行的坐标存到v变量里
	std::vector<cv::Point3f> v;
	std::ifstream ifs(file_name);
	std::string buffer;
	if (!ifs) {
		std::cout << "Open file fail" << std::endl;
		exit(1);
	}
	while (std::getline(ifs, buffer)) {
		std::istringstream ss;
		cv::Point3d tmp;
		ss.str(buffer);
		ss >> tmp.x >> tmp.y >> tmp.z;
		//std::cout << tmp << std::endl;
		v.push_back(tmp);
	}

	//把变量v里的数据输出到cloud变量
	cloud.width = v.size();
	cloud.height = 1; // This indicates that the point cloud is unorganized
	cloud.is_dense = false;
	cloud.resize(cloud.width * cloud.height);

	std::cout << cloud.width << std::endl;

	for (std::size_t i = 0; i < cloud.size(); ++i)
	{

		pcl::setFieldValue<pcl::PointXYZ, float>(cloud[i], x_idx, v[i].x);
		pcl::setFieldValue<pcl::PointXYZ, float>(cloud[i], y_idx, v[i].y);
		pcl::setFieldValue<pcl::PointXYZ, float>(cloud[i], z_idx, v[i].z);

	}

}


//读取双脚的stl文件路径,输出切割完左右脚stl文件
void splitfoots(string StlPath)
{
	//读取CAD模型
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(StlPath.c_str());
	reader->Update();
	//先转出到ply格式
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData = reader->GetOutput();
	polyData->GetNumberOfPoints();



	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	//从ply转pcd
	pcl::io::vtkPolyDataToPointCloud(polyData, *cloud2);
	cout << cloud2->size() << endl;


	string ascpath = "D:/work/scanner/footscan_tool/date/20220402091022.asc";
	readasc(ascpath, *cloud);


	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI; // The angle of rotation in radians
	float theta2 = M_PI / 2; // The angle of rotation in radians

	//X轴
	//transform_1(1, 1) = cos(theta);
	//transform_1(1, 2) = sin(theta);
	//transform_1(2, 1) = -sin(theta);
	//transform_1(2, 2) = cos(theta);

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



	//cout << "变换矩阵\n" << transform_1.matrix() << std::endl;
	//cout << "变换矩阵\n" << transform_2.matrix() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// 可以使用 transform_1 或 transform_2; t它们是一样的
	pcl::transformPointCloud(*cloud, *t_cloud, transform_1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	// 可以使用 transform_1 或 transform_2; t它们是一样的
	pcl::transformPointCloud(*t_cloud, *t_cloud2, transform_2);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(t_cloud2, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}



	cout << "开始聚类" << endl;
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



	cout << "完成聚类" << endl;

	// 遍历每一点云
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		//添加所有的点云到一个新的点云中
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(t_cloud2->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// 保存
		if (cluster->points.size() <= 0)
			break;

		vtkSmartPointer<vtkPolyData> polyData2 = vtkSmartPointer<vtkPolyData>::New();
		pcl::io::pointCloudTovtkPolyData(*cluster, polyData2);


		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cluster, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}


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

		if (currentClusterNum == 1)
			pcl::io::savePolygonFileSTL("D:/work/scanner/footscan_tool/date/L.stl", ret, true);
		else if (currentClusterNum == 2)
			pcl::io::savePolygonFileSTL("D:/work/scanner/footscan_tool/date/R.stl", ret, true);

		currentClusterNum++;
	}
}



int main(int argc, char** argv)
{
	string stlpath = "D:/work/scanner/footscan_tool/date/20220402091022.stl";
	splitfoots(stlpath);

	return 0;
}

