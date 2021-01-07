#include"pre_process.h"

Pre_process::Pre_process(std::string config_path_)
{
    //注意在这里，cloud_t不用加上Pre_process::
    cloud_t = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

	config_path=config_path_;

	FILE *fp = fopen(config_path_.c_str(), "r");

	if (fp == NULL)
	{
		printf("打开文件失败，使用默认参数\n");
		pass_through_z_f=1;
		pass_through_x_f=1;
		pass_through_y_f=1;

		table_remove_f=0;
		outlier_remove_f=0;
		surface_smooth_f=0;
		surface_subsample_f=0;
		z_min = 0.3;
		z_max =1.0;
		x_left = 0.1;
		x_right = 0.5;
		y_up = 0.1;
		y_down = 0.5;
        remain_rate = 0.4;
        remain_points = 500;
        k_points = 80;
        thresh =0.8;
        radius = 0.03;

		subsampling_leaf_size=0.008;
		//return -1;
	}
	else //从文件中读取
	{
		//检查文件中“”中的形式的文本，将数据读到对应的参数中
		//fscanf函数的作用是检查对应的字段，直到空格位置
        printf("打开文件成功，从文件读取点云预处理参数\n");
		fscanf(fp, "pass_through_x=%u\n", &pass_through_x_f);
		fscanf(fp, "pass_through_y=%u\n", &pass_through_y_f);
		fscanf(fp, "pass_through_z=%u\n", &pass_through_z_f);

		fscanf(fp, "table_remove=%u\n", &table_remove_f);
		fscanf(fp, "outlier_remove=%u\n", &outlier_remove_f);
		fscanf(fp, "surface_smooth=%u\n", &surface_smooth_f);
		fscanf(fp, "subsample=%u\n", &surface_subsample_f);


		fscanf(fp, "z_min=%f\n", &z_min);
		fscanf(fp, "z_max=%f\n", &z_max);
		fscanf(fp, "x_left=%f\n", &x_left);
		fscanf(fp, "x_right=%f\n", &x_right);
		fscanf(fp, "y_up=%f\n", &y_up);
		fscanf(fp, "y_down=%f\n", &y_down);
		
		fscanf(fp, "remain_rate=%f\n", &remain_rate);
		fscanf(fp, "remain_points=%d\n", &remain_points);
		fscanf(fp, "k_points=%d\n", &k_points);
		fscanf(fp, "thresh=%f\n", &thresh);
        fscanf(fp, "radius=%f\n", &radius);


        fscanf(fp, "subsampling_leaf_size=%f\n", &subsampling_leaf_size);
		fclose(fp);
	}
	// printf("直通滤波pass_min=%f  \n直通滤波pass_max=%f  \n平面剔除剩余比例 remain_rate = %f   \n平面剔除剩余点数 remain_points=%d  \n  离群点参考点数k_points=%d   
    // \n离群点废弃阈值thresh=%f  \n表面光滑参考半径radius= %f  \n", pass_min,pass_max,remain_rate,remain_points,k_points,thresh,radius);
	    //分割器初始化
    coefficients = boost::make_shared<pcl::ModelCoefficients>();
	inliers = boost::make_shared<pcl::PointIndices>();
 	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);
	extract.setNegative(true);

	//

	
}


void Pre_process::pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,const std::string &field_name)
{
	uint16_t cloud_in_num=cloud_in->size();
    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName (field_name);
	if(field_name=="z")
    	pass.setFilterLimits (z_min, z_max);
	else if(field_name=="x")
		pass.setFilterLimits (x_left, x_right);
	else if(field_name=="y")
		pass.setFilterLimits (y_up, y_down);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_out);
    printf("========直通滤波(%s)前点数：%d, 直通滤波后点数：%d========\n",field_name.c_str(),cloud_in_num, cloud_out->size());
}

void Pre_process::table_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
	uint16_t cloud_in_num=cloud_in->size();
    //printf("remain_rate=%f  \n remain_points=%d \n",remain_rate,remain_points);

	//===去除场景点云的支撑平面，并清除结果中的无效点===========================================
	unsigned nr_points = unsigned(cloud_in->points.size());
	//估计，剔除掉平面的点云，大约占所有点云的20%
	while (cloud_in->points.size() > remain_rate* nr_points)
	{
		seg.setInputCloud(cloud_in);
		seg.segment(*inliers, *coefficients);
		//PCL_INFO("Plane inliers: %u\n", inliers->indices.size());
		//如果上次剔除，虽然没有将点云降到20%，但是总数已经小于500了，就不要再剔除了
		if (inliers->indices.size() < remain_points) break;
		//开始剔除平面
		extract.setInputCloud(cloud_in);
		extract.setIndices(inliers);
		extract.filter(*cloud_out);
	}
		//剔除NANs
	pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, mapping);
	//PCL_INFO("Scene cloud dense?  %d \n", cloud_out->is_dense);
    printf("========平面剔除前点数：%d, 平面剔除后点数：%d========\n",cloud_in_num,cloud_out->size());

}

void Pre_process::outlier_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
	uint16_t cloud_in_num=cloud_in->size();
    //printf(" k_points=%d  \n thresh=%f \n",k_points,thresh);
    sor.setInputCloud(cloud_in);
    sor.setMeanK(k_points);    //设置参考范围
	sor.setStddevMulThresh(thresh);    //设置废弃阈值
	sor.filter(*cloud_out);
    printf("========离群剔除前点数：%d,   离群剔除后点数：%d========\n",cloud_in_num, cloud_out->size());

}
void Pre_process::subsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
	uint16_t cloud_in_num=cloud_in->size();
	Eigen::Vector4f subsampling_leaf_size_(subsampling_leaf_size,subsampling_leaf_size,subsampling_leaf_size,0);
	//subsampling_filter.setSaveLeafLayout(true);
	subsampling_filter.setInputCloud(cloud_in);
	subsampling_filter.setLeafSize(subsampling_leaf_size_);
	subsampling_filter.filter(*cloud_out);
	printf("========降采样前点数：%d, 降采样后点数：%d========\n",cloud_in_num, cloud_out->size());

}

void Pre_process::surface_smooth(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
	uint16_t cloud_in_num=cloud_in->size();
    printf(" radius=%f  \n",radius);
	mls.setInputCloud(cloud_in);  //设置输入点云
	mls.setPolynomialFit(true);//设置在最小二乘计算中需要法线估计
	mls.setSearchMethod(tree);
	mls.setSearchRadius(radius); //设置搜索半径，确定多项式拟合时所用的邻域点进行k近邻搜索时所用的半径
	// 曲面重建
	mls.process(mls_points);
	cloud_out->points.resize(mls_points.size());
	for (size_t i = 0; i < mls_points.size(); ++i)
	{
		cloud_out->points[i].x = mls_points.points[i].x;
		cloud_out->points[i].y = mls_points.points[i].y;
		cloud_out->points[i].z = mls_points.points[i].z;
	}
	cloud_out->width = mls_points.size();
	cloud_out->height = 1;
    printf("========光滑处理前点数：%d， 光滑处理后点数：%d========\n",cloud_in_num, cloud_out->size());


}

void Pre_process::pre_process(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
	FILE *fp = fopen(config_path.c_str(), "r");

	if (fp == NULL)
	{
		printf("打开文件失败，使用默认参数\n");
	}
	else //从文件中读取
	{
		fscanf(fp, "pass_through_x=%u\n", &pass_through_x_f);
		fscanf(fp, "pass_through_y=%u\n", &pass_through_y_f);
		fscanf(fp, "pass_through_z=%u\n", &pass_through_z_f);

		fscanf(fp, "table_remove=%u\n", &table_remove_f);
		fscanf(fp, "outlier_remove=%u\n", &outlier_remove_f);
		fscanf(fp, "surface_smooth=%u\n", &surface_smooth_f);
		fscanf(fp, "subsample=%u\n", &surface_subsample_f);


		fscanf(fp, "z_min=%f\n", &z_min);
		fscanf(fp, "z_max=%f\n", &z_max);
		fscanf(fp, "x_left=%f\n", &x_left);
		fscanf(fp, "x_right=%f\n", &x_right);
		fscanf(fp, "y_up=%f\n", &y_up);
		fscanf(fp, "y_down=%f\n", &y_down);
		
		fscanf(fp, "remain_rate=%f\n", &remain_rate);
		fscanf(fp, "remain_points=%d\n", &remain_points);
		fscanf(fp, "k_points=%d\n", &k_points);
		fscanf(fp, "thresh=%f\n", &thresh);
        fscanf(fp, "radius=%f\n", &radius);


        fscanf(fp, "subsampling_leaf_size=%f\n", &subsampling_leaf_size);
		fclose(fp);
	}

    printf("=================开始点云预处理=================\n");
	if(pass_through_x_f==1)
		Pre_process::pass_through(cloud_in,cloud_in,"x");
	if(pass_through_y_f==1)
	Pre_process::pass_through(cloud_in,cloud_in,"y");
	if(pass_through_z_f==1)
	Pre_process::pass_through(cloud_in,cloud_in,"z");

	if(table_remove_f==1)
		Pre_process::table_remove(cloud_in,cloud_in);
	if(outlier_remove_f==1)
		Pre_process::outlier_remove(cloud_in,cloud_in);
	if(surface_smooth_f==1)
		Pre_process::surface_smooth(cloud_in, cloud_in);

	cloud_out=cloud_in->makeShared();
}

void Pre_process::pre_process(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
{
	FILE *fp = fopen(config_path.c_str(), "r");

	if (fp == NULL)
	{
		printf("打开文件失败，使用默认参数\n");
	}
	else //从文件中读取
	{
		fscanf(fp, "pass_through_x=%u\n", &pass_through_x_f);
		fscanf(fp, "pass_through_y=%u\n", &pass_through_y_f);
		fscanf(fp, "pass_through_z=%u\n", &pass_through_z_f);

		fscanf(fp, "table_remove=%u\n", &table_remove_f);
		fscanf(fp, "outlier_remove=%u\n", &outlier_remove_f);
		fscanf(fp, "surface_smooth=%u\n", &surface_smooth_f);
		fscanf(fp, "subsample=%u\n", &surface_subsample_f);


		fscanf(fp, "z_min=%f\n", &z_min);
		fscanf(fp, "z_max=%f\n", &z_max);
		fscanf(fp, "x_left=%f\n", &x_left);
		fscanf(fp, "x_right=%f\n", &x_right);
		fscanf(fp, "y_up=%f\n", &y_up);
		fscanf(fp, "y_down=%f\n", &y_down);
		
		fscanf(fp, "remain_rate=%f\n", &remain_rate);
		fscanf(fp, "remain_points=%d\n", &remain_points);
		fscanf(fp, "k_points=%d\n", &k_points);
		fscanf(fp, "thresh=%f\n", &thresh);
        fscanf(fp, "radius=%f\n", &radius);


        fscanf(fp, "subsampling_leaf_size=%f\n", &subsampling_leaf_size);
		fclose(fp);
	}


    printf("=================开始点云预处理=================\n");
	if(pass_through_x_f==1)
		Pre_process::pass_through(cloud_in,cloud_in,"x");
	if(pass_through_y_f==1)
	Pre_process::pass_through(cloud_in,cloud_in,"y");
	if(pass_through_z_f==1)
	Pre_process::pass_through(cloud_in,cloud_in,"z");

	if(table_remove_f==1)
		Pre_process::table_remove(cloud_in,cloud_in);
	if(outlier_remove_f==1)
		Pre_process::outlier_remove(cloud_in,cloud_in);
	if(surface_smooth_f==1)
		Pre_process::surface_smooth(cloud_in, cloud_in);
	if(surface_subsample_f==1)
		Pre_process::subsample(cloud_in, cloud_in);


}