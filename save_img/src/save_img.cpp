#include <ros/ros.h>
#include <termios.h>

// #include <pcl_ros/point_cloud.h>
// #include <pcl/io/ply_io.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/ros/conversions.h>

// #include <pcl/visualization/cloud_viewer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <string>
#include <sstream>

// PCL specific includes

int num;

// bool pc_flg;

cv_bridge::CvImagePtr rgb_image;
// sensor_msgs::PointCloud2ConstPtr pc_image;

//pcl::visualization::CloudViewer viewerr("Simple Cloud Viewerr");

char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)  ;
		//ROS_INFO("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

// void get_pc(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
// {
//   pc_flg = true;
//   pc_image = pc_msg;
// }

void get_rgb(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  rgb_image = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);

//   if(pc_flg == false) return;

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromROSMsg(*pc_image, *cloud);

  /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud,*cloud_RGB);*/

  int c = getch();   // call your non-blocking input function
  if (c == 's')
  {
    std::ostringstream num_str;
    num_str << num;

    std::string rgb_str = "/home/kist/workspace/" + num_str.str() + "_rgb.png";
    // std::string pc_str = "/workspace/" + num_str.str() + "_xyz.ply";
	//std::string pc_rgb_str = "/home/kist/workspace/" + num_str.str() + "_rgb.ply";
    cv::imwrite(rgb_str,rgb_image->image);

    // pcl::PLYWriter writer;
    // writer.write(pc_str,*cloud);
	//writer.write(pc_rgb_str,*cloud_RGB);

	std::cout << "-------------- SAVE ! ------------------" << num << " -----------" << std::endl;
    num++;
  }

  //viewerr.showCloud (cloud);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "save_img_node");
  ros::NodeHandle nh;
  
  num = 0;	
//   pc_flg = false;
  
//   ros::Subscriber sub3 = nh.subscribe ("pc_img", 1, get_pc);
  ros::Subscriber sub = nh.subscribe ("rgb_img", 1, get_rgb);

  ros::spin();
}
