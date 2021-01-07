//PCL 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/mls.h>

class Pre_process
{
    private:
    //剔除NAN点容器
	std::vector<int> mapping;
    //平面分割器
	pcl::SACSegmentation<pcl::PointXYZ> seg;
    //抽取器
    pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointIndices::Ptr inliers;
    //剔除离群点
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //表面光滑
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    boost::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>> tree;
    pcl::PointCloud<pcl::PointNormal> mls_points;
     //直通滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    //降采样滤波器
    pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;
	//***************************************滤波参数********************************************
    //滤波器使能
    uint16_t pass_through_x_f,pass_through_y_f,pass_through_z_f,table_remove_f,outlier_remove_f,surface_smooth_f,surface_subsample_f;
	//直通滤波
    float z_min,z_max,x_left,x_right,y_up,y_down;
	//--------------去除平面
	//剩余点比例阈值
	float  remain_rate;
	//剩余点数阈值
	uint16_t remain_points;
	//--------------离群点剔除
	//周边点的参考数量
	uint16_t k_points;
	//废弃阈值
	float thresh;
	//--------------表面光滑
	//搜索半径，光滑半径
	float radius;

    float subsampling_leaf_size;

    std::string config_path;



    public:
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_t;


    Pre_process(std::string config_path_);
    /*
	*brief：直通滤波处理
    *input:       pcl::PointCloud<pcl::PointXYZ>::Ptr     cloud
    *output:    一个中间点云，内部点云cloud_t
	*/
    void pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,const std::string &field_name);
    /*
	*brief：降采样
    *input:       pcl::PointCloud<pcl::PointXYZ>::Ptr     cloud
    *output:    一个中间点云，内部点云cloud_t
	*/
    void subsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);


    /*
	*brief：剔除支撑平面
    *input:       pcl::PointCloud<pcl::PointXYZ>::Ptr     cloud
    *output:    没有桌子等平面的cloud
	*/
    void table_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
    /*
	*brief：离群点剔除
    *input:       pcl::PointCloud<pcl::PointXYZ>::Ptr     cloud
    *output:    离群点剔除之后的cloud
	*/

    void outlier_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
    /*
	*brief：表面光滑处理
    *input:       pcl::PointCloud<pcl::PointXYZ>::Ptr     cloud
    *output:    表面光滑处理之后的cloud
	*/
    void surface_smooth(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    void pre_process(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    void pre_process(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);



};