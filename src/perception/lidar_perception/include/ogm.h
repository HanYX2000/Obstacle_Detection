#ifndef OGM_H
#define OGM_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloud(   new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud(  new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr rs_cloud(     new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud( new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());    

Matrix4d left_to_rs;
Matrix4d right_to_rs;

ros::Publisher pub_all;
ros::Publisher pub_target;
ros::Publisher pub_map;
ros::Publisher pub_pedmap;	

nav_msgs::OccupancyGrid mapExtern;

double L1;
double L2;
double W1;
double W2;
double H1;
double H2;

double ego_left;
double ego_right;
double ego_front;
double ego_back;
double ego_top;
double ego_bottom;

double resolution;
double height_diff;

float floatMax = numeric_limits<float>::max();  
float floatMin = -numeric_limits<float>::max();

//是否跟踪反光板
bool follow_target = false;

void left_callback(const sensor_msgs::PointCloud2ConstPtr &left_msg) 
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*left_msg,*cloud);
	pcl::transformPointCloud (*cloud,*cloud, left_to_rs);
	left_cloud->points.clear();
	for (auto it = cloud->points.begin(); it < cloud->points.end();it++)
        {
		if (abs(it->x)<30 && abs(it->y)<30)
		{
			left_cloud->points.push_back(*it);
		}
	}
}

void right_callback(const sensor_msgs::PointCloud2ConstPtr &right_msg) 
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*right_msg,*cloud);
	pcl::transformPointCloud (*cloud,*cloud, right_to_rs);
	right_cloud->points.clear();
	for (auto it = cloud->points.begin(); it < cloud->points.end();it++)
        {
		if (abs(it->x)<30 && abs(it->y)<30)
		{
			right_cloud->points.push_back(*it);
		}
	}
}

void rs_callback(const sensor_msgs::PointCloud2ConstPtr &rs_msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*rs_msg,*cloud);
	rs_cloud->points.clear();

	int rows=(W2-W1)/resolution;
	int cols=(L2-L1)/resolution;
	float** maxheight=new float*[rows];
	float** minheight=new float*[rows];
	for(int i=0;i<rows;i++)
	{
		maxheight[i]=new float[cols];
		minheight[i]=new float[cols];
		for(int j=0;j<cols;j++)
		{
			maxheight[i][j]=floatMin;
			minheight[i][j]=floatMax;
		}
	}

	// 将点云保存至rs_cloud,并记录点云中的最高点和最低点
	for (auto it = cloud->points.begin(); it < cloud->points.end();it++)
    {
		// 去除车身点云、ROI区域之外的点云
        if ((it->x<ego_front && it->x>ego_back && it->y>ego_right && it->y<ego_left) || (it->z>ego_top) || (it->z<ego_bottom)){continue;}
        rs_cloud->points.push_back(*it);
		if(it->x>L1 && it->x<L2 && it->y>W1 && it->y<W2 && it->z>H1 && it->z<H2)
		{
			int x=(it->x-L1)/resolution;
			int y=(it->y-W1)/resolution;
			//cout<<"rs: "<<x<<" "<<y<<" "<<rows<<" "<<cols<<endl;
			if(it->z>maxheight[y][x]){maxheight[y][x]=it->z;}
			if(it->z<minheight[y][x]){minheight[y][x]=it->z;}
		}
	}

	for (auto it = left_cloud->points.begin(); it < left_cloud->points.end();it++)
    {
        if ((it->x<ego_front && it->x>ego_back && it->y>ego_right && it->y<ego_left) || (it->z>ego_top) || (it->z<ego_bottom)){continue;}
		rs_cloud->points.push_back(*it);
		if(it->x>L1 && it->x<L2 && it->y>W1 && it->y<W2 && it->z>H1 && it->z<H2)
		{
			int x=(it->x-L1)/resolution;
			int y=(it->y-W1)/resolution;
			//cout<<"left: "<<x<<" "<<y<<" "<<rows<<" "<<cols<<endl;
			if(it->z>maxheight[y][x]){maxheight[y][x]=it->z;}
			if(it->z<minheight[y][x]){minheight[y][x]=it->z;}
		}
	}

	for (auto it = right_cloud->points.begin(); it < right_cloud->points.end();it++)
    {
        if ((it->x<ego_front && it->x>ego_back && it->y>ego_right && it->y<ego_left) || (it->z>ego_top) || (it->z<ego_bottom)){continue;}
		rs_cloud->points.push_back(*it);
		if(it->x>L1 && it->x<L2 && it->y>W1 && it->y<W2 && it->z>H1 && it->z<H2)
		{
			int x=(it->x-L1)/resolution;
			int y=(it->y-W1)/resolution;
			if(it->z>maxheight[y][x]){maxheight[y][x]=it->z;}
			if(it->z<minheight[y][x]){minheight[y][x]=it->z;}
		}
	}

	// 发布all_lidars点云
	rs_cloud->width=rs_cloud->points.size();
	rs_cloud->height=1;
	sensor_msgs::PointCloud2 output_all;
	string frame_id="rslidar";
	pcl::toROSMsg (*rs_cloud, output_all);
	output_all.header.frame_id = frame_id;
	// pub_all.publish (output_all);

	// 设置栅格地图
	nav_msgs::OccupancyGrid map;
	map.header.frame_id = frame_id;
	map.header.stamp = rs_msg->header.stamp;
	map.info.resolution = resolution;
	map.info.width = cols;
	map.info.height = rows;
	map.info.origin.position.x = L1;
	map.info.origin.position.y = W1;
	map.data.resize(rows*cols);

    // 画出所有被占据的栅格
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			if((maxheight[i][j]>minheight[i][j]) && (maxheight[i][j]-minheight[i][j]>height_diff))
			{
				int index = j+map.info.width*i;
				// map.data[index]=int((maxheight[i][j]-minheight[i][j])*100);	
                map.data[index] = (maxheight[i][j] == minheight[i][j]) ? 0 : 100;
			}
		}
	}

	// 把车在图中画出来
	int car_left = (ego_left - W1) / resolution;
	int car_right = (ego_right - W1) / resolution;
	int car_front = (ego_front - L1) / resolution;
	int car_back = (ego_back - L1) / resolution;
	for(int car_y = car_right; car_y <= car_left; car_y++)
	{
		for(int car_x = car_back; car_x <= car_front; car_x++)
		{
			int car_index = car_x + car_y * map.info.width;
			map.data[car_index] = 127;
		}
	}

	// 接收到开始跟踪指令后，识别反光条并画图
	if(follow_target)
	{
		// 先将反射强度高的位置取出来
		for (auto it = rs_cloud->points.begin(); it < rs_cloud->points.end(); it++)
		{
			if (it->intensity < 150) {continue;}
			target_cloud->points.push_back(*it);
		}
		sensor_msgs::PointCloud2 output_target;
		string frame_id="rslidar";
		pcl::toROSMsg (*target_cloud, output_target);
		output_target.header.frame_id = frame_id;
		pub_all.publish(output_target);
		// kdtree cluster target_cloud
    	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
		if(target_cloud->points.size() != 0)
		{
			tree -> setInputCloud(target_cloud);
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
			ec.setClusterTolerance(0.2);
			ec.setMinClusterSize(1);
			ec.setMaxClusterSize(10000);
			ec.setSearchMethod(tree);
			ec.setInputCloud(target_cloud);
			ec.extract(cluster_indices);

			int max_size = 0;
			int max_id = 0;
			for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
			{
				int size = it->indices.end() - it->indices.begin();
				if(size > max_size)
				{
					cluster_cloud->points.clear();
					max_size = size;
					for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
					{
						cluster_cloud->points.push_back(target_cloud->points[*pit]);
						cluster_cloud->width = cluster_cloud->points.size();
						cluster_cloud->height = 1;
						cluster_cloud->is_dense = true;
					}
				}
			}

			// 发布反光条点云
			sensor_msgs::PointCloud2 output_target;
			pcl::toROSMsg (*cluster_cloud, output_target);
			output_target.header.frame_id = "rslidar";
			pub_target.publish (output_target);

			// 解析高反射强度对应的坐标范围，把目标反光条画出来
			// float target_low_x = 0;
			// float target_up_x  = 0;
			// float target_low_y = 0;
			// float target_up_y  = 0;
			// for (auto it = cluster_cloud->points.begin(); it < cluster_cloud->points.end(); it++)
			// {
			// 	target_low_x = (it->x < target_low_x) ? it->x : target_low_x;
			// 	target_up_x  = (it->x > target_up_x) ? it->x : target_up_x;
			// 	target_low_y = (it->y < target_low_y) ? it->y : target_low_y;
			// 	target_up_y  = (it->y > target_up_y) ? it->y : target_up_y;
			// }
			// int target_start_x = (target_low_x - L1) / resolution;
			// int target_end_x   = (target_up_x - L1) / resolution;
			// int target_start_y = (target_low_y - W1) / resolution;
			// int target_end_y   = (target_up_y - W1) / resolution;

			int target_center_x = (cluster_cloud->points.begin()->x - L1) / resolution;
			int target_center_y = (cluster_cloud->points.begin()->y - W1) / resolution;

			// for(int y = target_start_y; y <= target_end_y; y++)
			// {
			// 	for(int x = target_start_x; x <= target_end_x; x++)
			// 	{
			// 		int target_index = x + y * map.info.width;
			// 		map.data[target_index] = -64;
			// 	}
			// }

			for(int y = target_center_y - (2/resolution); y <= target_center_y + (2/resolution); y++)
			{
				for(int x = target_center_x - (2/resolution); x <= target_center_x + (2/resolution); x++)
				{
					int index = x + y * map.info.width;
					map.data[index] = -64;
				}
			}	
		}
		target_cloud->points.clear();
	}
	
	// pub_map.publish(map);
	mapExtern = map;
}

void obs_callback(const geometry_msgs::PoseArray::ConstPtr &obs_msg)
{
	if(obs_msg->poses.size() != 0)
	{
		int num_target = obs_msg->poses[0].position.z;  //行人、车辆总量

		for(int id = 0; id < num_target; id++)
		{
			int center_x = (obs_msg->poses[id].position.x - L1) / resolution;
			int center_y = (obs_msg->poses[id].position.y - W1) / resolution;
			//标出行人周围3x3区域
			if(obs_msg->poses[id].orientation.x==0) //行人
			{
				for(int y = center_y - 1; y <= center_y + 1; y++)
				{
					for(int x = center_x - 1; x <= center_x + 1; x++)
					{
						int index = x + y * mapExtern.info.width;
						mapExtern.data[index] = -128;
					}
				}	
			}
			//标出车辆周围区域 此处待改进***（按照方向绘制ogm）
			else if (obs_msg->poses[id].orientation.x==1) //车辆
			{
				float x_ = obs_msg->poses[id].position.x;
				float y_ = obs_msg->poses[id].position.y;
				float w_ = obs_msg->poses[id].orientation.y;
				float l_ = obs_msg->poses[id].orientation.z;
				float yaw_ = obs_msg->poses[id].orientation.w;
				vector<float> corner1 = {x_ + cos(yaw_) * l_/2 - sin(yaw_) * w_/2, y_ + sin(yaw_) * l_/2 + cos(yaw_) * w_/2};
				vector<float> corner2 = {x_ + cos(yaw_) * l_/2 + sin(yaw_) * w_/2, y_ + sin(yaw_) * l_/2 - cos(yaw_) * w_/2};
				vector<float> corner3 = {x_ - cos(yaw_) * l_/2 - sin(yaw_) * w_/2, y_ - sin(yaw_) * l_/2 + cos(yaw_) * w_/2};
				vector<float> corner4 = {x_ - cos(yaw_) * l_/2 + sin(yaw_) * w_/2, y_ - sin(yaw_) * l_/2 - cos(yaw_) * w_/2};
				int max_X = (max(max(corner1[0], corner2[0]), max(corner3[0], corner4[0])) - L1) / resolution;
				int min_X = (min(min(corner1[0], corner2[0]), min(corner3[0], corner4[0])) - L1) / resolution;
				int max_Y = (max(max(corner1[1], corner2[1]), max(corner3[1], corner4[1])) - W1) / resolution;
				int min_Y = (min(min(corner1[1], corner2[1]), min(corner3[1], corner4[1])) - W1) / resolution;

				for(int y = min_Y; y <= max_Y; y++)
				{
					for(int x = min_X; x <= max_X; x++)
					{
						if((cos(yaw_)*(y-y_)-sin(yaw_)*(x-x_)-w_/2)<0 && (cos(yaw_)*(y-y_)-sin(yaw_)*(x-x_)+w_/2)>0 && (sin(yaw_)*(y-y_)+cos(yaw_)*(x-x_)-l_/2)<0 && (sin(yaw_)*(y-y_)+cos(yaw_)*(x-x_)+l_/2)>0)
						{
							int index = x + y * mapExtern.info.width;
							mapExtern.data[index] = -32;
						}
					}
				}	
			}
		}
	}
	pub_pedmap.publish(mapExtern);
}

void ped_callback(const geometry_msgs::PoseArray::ConstPtr &ped_msg)
{
	int num_target = ped_msg->poses[-1].position.z;  //行人总量
	for(int id = 0; id < num_target; id++)
	{
		int center_x = (ped_msg->poses[id].position.x - L1) / resolution;
		int center_y = (ped_msg->poses[id].position.y - W1) / resolution;

		//将周围3x3区域内被占用的格子全部标为行人
		for(int y = center_y - 1; y <= center_y + 1; y++)
		{
			for(int x = center_x - 1; x <= center_x + 1; x++)
			{
				int index = x + y * mapExtern.info.width;
				if(mapExtern.data[index]==100)  //若被占用(data=100)
				{
					mapExtern.data[index] = -128;
				}
			}
		}	
		
	}
}

#endif
