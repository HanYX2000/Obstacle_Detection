#include<ogm.h>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> ApproximatePolicy; //近似同步策略

int main(int argc, char** argv){

	ros::init(argc, argv, "ogm");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(20);
	// pub PointCloud
	pub_all = nh.advertise<sensor_msgs::PointCloud2> ("/all_lidars", 1);
	pub_target = nh.advertise<sensor_msgs::PointCloud2> ("/target_man", 1);
	// pub OccupancyGrid
	pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid_map", 1);
    pub_pedmap = nh.advertise<nav_msgs::OccupancyGrid>("/OGM_Final", 1);

	//ros::Subscriber left_sub = nh.subscribe("/left/velodyne_points", 1, left_callback);
	//ros::Subscriber right_sub = nh.subscribe("/right/velodyne_points", 1, right_callback);
	ros::Subscriber rs_sub = nh.subscribe("/rslidar_points", 1, rs_callback);
    //Get Ped Msg 
    ros::Subscriber obs_sub = nh.subscribe("/obstacle_poses", 1000, obs_callback);
	ros::Subscriber ped_sub = nh.subscribe("/Cam_Ped_poses", 1000, ped_callback);

	left_to_rs<< 0.99906056,  0.0391486,  -0.01855819, -0.2308354, 
	-0.01834761,  0.77035743,  0.63734865,  0.57236422,
	0.03924774, -0.63640965,  0.77035292, -0.15694635,
	0.,          0.,          0.,          1.;   
	right_to_rs<<9.99965884e-01, -5.11903045e-03,  6.41002022e-03, -2.12338835e-01,
	8.20290843e-03,  6.17173329e-01, -7.86784760e-01, -6.06582964e-01,
	7.14748321e-05,  7.86810783e-01,  6.17194128e-01, -1.75552731e-01,
	0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;

	nh.param("/ogm/L1", L1, -40.0);
	nh.param("/ogm/L2", L2, 40.0);
	nh.param("/ogm/W1", W1, -40.0);
	nh.param("/ogm/W2", W2, 40.0);
	nh.param("/ogm/H1", H1, -2.6);
	nh.param("/ogm/H2", H2, 0.2);
	nh.param("/ogm/resolution", resolution, 0.2);
	nh.param("/ogm/height_diff", height_diff, 0.2 );

	nh.param("/ogm/ego_left", ego_left, 1.2);
	nh.param("/ogm/ego_right", ego_right, -1.1);
	nh.param("/ogm/ego_top", ego_top, 0.3);
	nh.param("/ogm/ego_bottom", ego_bottom, -2.0);
	nh.param("/ogm/ego_front", ego_front, 2.9);
	nh.param("/ogm/ego_back", ego_back, -1.8);

	loop_rate.sleep();
	ros::spin();
	return 0;

}
