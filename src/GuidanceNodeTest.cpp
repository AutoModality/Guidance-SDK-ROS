/*
 * main_sdk0428.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: craig
 */

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <dji_sdk/RCChannels.h>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
#include <bag_logger.h>
#include <vb_main.h>

BagLogger *BagLogger::s_instance_ = 0;

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Subscriber depth_image_sub;
ros::Subscriber imu_sub;
ros::Subscriber velocity_sub;
ros::Subscriber obstacle_distance_sub;
ros::Subscriber ultrasonic_sub;
ros::Subscriber position_sub;
ros::Subscriber rc_channels_sub;

bool video_out = false;

using namespace cv;
#define WIDTH 320
#define HEIGHT 240

/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    LOG_MSG("/guidance/left_image", left_img, 2);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (video_out)
    {
        cv::imshow("left_image", cv_ptr->image);
        cv::waitKey(1);
    }
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img)
{
    LOG_MSG("/guidance/right_image", right_img, 3);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (video_out)
    {
        cv::imshow("right_image", cv_ptr->image);
        cv::waitKey(1);
    }
}

/* depth greyscale image */
void depth_image_callback(const sensor_msgs::ImageConstPtr& depth_img)
{
    LOG_MSG("/guidance/depth_image", depth_img, 2);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (video_out)
    {
        cv::Mat depth8(HEIGHT, WIDTH, CV_8UC1);
        cv_ptr->image.convertTo(depth8, CV_8UC1);
        cv::imshow("depth_image", depth8);
        cv::waitKey(1);
    }
}

/* imu */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
    LOG_MSG("/guidance/imu", g_imu, 3);
    //ROS_INFO_THROTTLE(2, "frame_id: %s stamp: %d", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
    ROS_INFO_THROTTLE(2, "imu: [%f %f %f %f %f %f %f]", g_imu.transform.translation.x, g_imu.transform.translation.y, g_imu.transform.translation.z, 
						g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z, g_imu.transform.rotation.w );
}

/* velocity */
void velocity_callback(const geometry_msgs::Vector3Stamped& g_vo)
{ 
    LOG_MSG("/guidance/velocity", g_vo, 1);
    //ROS_INFO_THROTTLE(2, "frame_id: %s stamp: %d", g_vo.header.frame_id.c_str(), g_vo.header.stamp.sec );
    ROS_INFO_THROTTLE(2, "velocity: [%f %f %f]", g_vo.vector.x, g_vo.vector.y, g_vo.vector.z );
}

double ob_range[5];
int ul_intensity[5];
double ul_range[5];
ros::Time stamp;

void print_Ul_OBS() {
    ROS_INFO_THROTTLE(1, "ul:int:obs D[%05.3f : %d : %05.3f] F[%05.3f : %d : %05.3f] R[%05.3f : %d : %05.3f] B[%05.3f : %d : %05.3f] L[%05.3f : %d : %05.3f] ",
         ul_range[0], ul_intensity[0], ob_range[0],
         ul_range[1], ul_intensity[1], ob_range[1],
         ul_range[2], ul_intensity[2], ob_range[2],
         ul_range[3], ul_intensity[3], ob_range[3],
         ul_range[4], ul_intensity[4], ob_range[4]);
}

/* obstacle distance */
void obstacle_distance_callback(const sensor_msgs::LaserScan& g_oa)
{
    for (int i = 0; i < 5; i++)
    {
        ob_range[i] = g_oa.ranges[i];
    }
    LOG_MSG("/guidance/obstacle_distance", g_oa, 1);
    //ROS_INFO_THROTTLE(2, "frame_id: %s stamp: %d", g_oa.header.frame_id.c_str(), g_oa.header.stamp.sec );
//    ROS_INFO_THROTTLE(2, "obstacle distance: [%f %f %f %f %f]\n", g_oa.ranges[0], g_oa.ranges[1], g_oa.ranges[2], g_oa.ranges[3], g_oa.ranges[4] );
}
/* ultrasonic */
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
    for (int i = 0; i < 5; i++)
    {
        ul_range[i] = g_ul.ranges[i];
        ul_intensity[i] = (int)g_ul.intensities[i];
    }
    LOG_MSG("/guidance/ultrasonic", g_ul, 1);
    //ROS_INFO_THROTTLE(2, "frame_id: %s stamp: %d", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );

//    ROS_INFO_THROTTLE(2, "ultrasonic distance: [%f : %d] [%f : %d] [%f : %d] [%f : %d] [%f : %d] ",
//            g_ul.ranges[0], (int)g_ul.intensities[0],
//            g_ul.ranges[1], (int)g_ul.intensities[1],
//            g_ul.ranges[2], (int)g_ul.intensities[2],
//            g_ul.ranges[3], (int)g_ul.intensities[3],
//            g_ul.ranges[4], (int)g_ul.intensities[4]);
    print_Ul_OBS();
}

/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
	printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
	for (int i = 0; i < 5; i++)
		printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
}

enum class DJI_RC_MODE {POS=0, ALT, API};
enum class DJI_RC_GEAR {UP=0, DOWN, UNKNOWN};
DJI_RC_MODE rc_mode_ {DJI_RC_MODE::POS};
DJI_RC_GEAR rc_gear_ {DJI_RC_GEAR::UNKNOWN};

void djiRemoteCB(const dji_sdk::RCChannels::ConstPtr&  msg) {
    if (msg->mode < -10)
    {
        if (rc_mode_ != DJI_RC_MODE::POS)
        {
            ROS_INFO("GTN: CHANGING TO POS MODE");
        }
        rc_mode_ = DJI_RC_MODE::POS;
    }
    else if (msg->mode == 0)
    {
        if (rc_mode_ != DJI_RC_MODE::ALT)
        {
            ROS_INFO("GTN: CHANGING TO ALT MODE");
        }
       rc_mode_ = DJI_RC_MODE::ALT;
    }
    else
    {
        if (rc_mode_ != DJI_RC_MODE::API)
        {
            ROS_INFO("GTN: CHANGING TO API MODE");
        }
        rc_mode_ = DJI_RC_MODE::API;
    }

    if (msg->gear < -6000)
    {
        // gear switch up
        if (rc_gear_ == DJI_RC_GEAR::DOWN)
        {
            ROS_INFO("GTN: CHANGING GEAR UP, START LOGGING");
            BagLogger::instance()->startLogging("GT",2);
        }
        rc_gear_ = DJI_RC_GEAR::UP;
    }
    else
    {
        // gear switch down
        if (rc_gear_ == DJI_RC_GEAR::UP)
        {
            ROS_INFO("GTN: CHANGING GEAR DOWN, STOP LOGGING");
            BagLogger::instance()->stopLogging();
        }
        rc_gear_ = DJI_RC_GEAR::DOWN;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;

//    left_image_sub        = my_node.subscribe("/guidance/left_image",  10, left_image_callback);
//    right_image_sub       = my_node.subscribe("/guidance/right_image", 10, right_image_callback);
//    depth_image_sub       = my_node.subscribe("/guidance/depth_image", 10, depth_image_callback);
//    imu_sub               = my_node.subscribe("/guidance/imu", 1, imu_callback);
//    velocity_sub          = my_node.subscribe("/guidance/velocity", 1, velocity_callback);
    obstacle_distance_sub = my_node.subscribe("/guidance/obstacle_distance", 1, obstacle_distance_callback);
	ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);
//	position_sub = my_node.subscribe("/guidance/position", 1, position_callback);
	rc_channels_sub = my_node.subscribe<dji_sdk::RCChannels>("/dji_sdk/rc_channels", 10, djiRemoteCB);

    bool start_logging = false;
    ros::Time st = ros::Time::now();

	if (my_node.hasParam("video"))
	{
            ROS_INFO("VIDEO OUTPUT ENABLED");
	    video_out = true;
	}
        else
	{
            ROS_INFO("VIDEO OUTPUT DISABLED");
	    video_out = false;
	}
        

	bool debug_on = userInputEnabled(my_node);
	ROS_INFO("GTN: USER INPUT %s", debug_on ? "ENABLED" : "DISABLED");

	if (!debug_on)
	{
		ros::spin();
	}
	else
	{
		changemode(1);

		ros::Rate loop_rate(10);

		while (ros::ok())
		{
			// get the next event from the keyboard
			while(kbhit())
			{
				int c = getchar();

				switch (c)
				{
				case 'O':
				case 'o':
                			ROS_INFO("LOGGING ON");		
                			BagLogger::instance()->startLogging("GT",2);
					break;
				case 'F':
				case 'f':
                			ROS_INFO("LOGGING OFF");		
                			BagLogger::instance()->stopLogging();
					break;
				case 'h':
					printf("GUIDANCE TEST\n");
					printf("o - logging on\n");
					printf("f - logging off\n");
					break;
				}

			}

		    ros::spinOnce();

		    loop_rate.sleep();
		}

		changemode(0);
	}

    return 0;
}
