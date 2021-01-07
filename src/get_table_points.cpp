#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
//PCL 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>

#include"pre_process.h"

using namespace std;
using namespace pcl;



class RAI_pre_precess
{
	public:
  	ros::NodeHandle nh;

		ros::Publisher pub;
    ros::Publisher pub_1;

		
		ros::Subscriber sub;

    tf::TransformListener listener;

    Eigen::Matrix4f transform_matrix;

        //预处理类
	    Pre_process pre_process;

	    sensor_msgs::PointCloud2 pre_pocessed;//声明的输出的点云的格式


    RAI_pre_precess(std::string config_path):pre_process(config_path)
    {
      
      tf::StampedTransform transform;
      bool tf_ok=false;
      while(!tf_ok){
        try{
          listener.lookupTransform("/ar_marker_6", "/kinect2_rgb_optical_frame",
                                  ros::Time(0), transform);
          tf_ok=true;
          TransformToMatrix(transform,transform_matrix);
        }
        catch (tf::TransformException &ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
      }





		  pub = nh.advertise<sensor_msgs::PointCloud2> ("/table_top_points", 1);
      pub_1 = nh.advertise<sensor_msgs::PointCloud2> ("/table_top_points_subsampled", 1);
      sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/qhd/points", 1, &RAI_pre_precess::cloud_cb,this);




    }

    
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_subsampled(new PointCloud<PointXYZ>());
		PointCloud<PointXYZ>::Ptr cloud_transformed(new PointCloud<PointXYZ>());

    //print();
    printf("/kinect2/hd/points的父坐标系为  %s \n", cloud_msg->header.frame_id.c_str());
		//把kinect点云数据转化为pcl数据
		pcl::fromROSMsg	(*cloud_msg,*cloud);
		//copyPointCloud(*cloud, *cloud_copy);

		//进行预处理
		pre_process.pre_process(cloud);
    //降采样单独拿出来
    pre_process.subsample(cloud,cloud_subsampled);

    pcl::transformPointCloud(*cloud,*cloud_transformed,transform_matrix);

    //预处理发布
		pcl::toROSMsg	(*cloud_transformed, pre_pocessed);//第一个参数是输入，后面的是输出

    //设置处理后的消息
    pre_pocessed.header.frame_id="/ar_marker_6";
    pre_pocessed.header.stamp = ros::Time::now();
    pre_pocessed.header.seq=1;

    //发布出去
    pub.publish (pre_pocessed);

    pcl::transformPointCloud(*cloud_subsampled,*cloud_transformed,transform_matrix);
    //预处理发布
		pcl::toROSMsg	(*cloud_transformed, pre_pocessed);//第一个参数是输入，后面的是输出

    //设置处理后的消息
    pre_pocessed.header.frame_id="/ar_marker_6";
    pre_pocessed.header.stamp = ros::Time::now();
    pre_pocessed.header.seq=1;
    //发布出去
    pub_1.publish (pre_pocessed);

    }

    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) 
    {
        Eigen::Translation3f tl_btol(
        transform.getOrigin().getX(), 
        transform.getOrigin().getY(), 
        transform.getOrigin().getZ());
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
        Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
        transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
        return true;
    }



};

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "table_top_points");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  //ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  //RAI_pre_precess pre_process("../config/prepocess_prarm.txt");
  if (argc==2)  
  {
    RAI_pre_precess pre_process(argv[1]);
    
    ros::spin();
  }
  else
  {
    printf("参数过多，或未输入配置文件地址！\n");
  }
  

  return 0;
}