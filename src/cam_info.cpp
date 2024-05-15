#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "camera_info_publisher");
    ros::NodeHandle nh("~");

    //load params
    std::string calibration_path, camera_name;
    nh.param("calibration_path", calibration_path, std::string(""));
    nh.param("camera_name", camera_name, std::string(""));

    //check params
    if(calibration_path.empty())
       // ROS_INFO_STREAM("Camera Calibration URL: " << calibration_path);
    //else
    {
        ROS_ERROR_STREAM("Failed to get calibration file from " << calibration_path);
    }
    if(!camera_name.empty())
        ROS_INFO_STREAM("Camera Name: " << camera_name);
    else
    {
        ROS_ERROR_STREAM("Failed to get camera name, using 'camera' ");
        camera_name="camera";
    }

    // Load camera calibration parameters from calibration file
    sensor_msgs::CameraInfo camera_info_msg;
    camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name);
    if (!cam_info_manager.loadCameraInfo(calibration_path)) {
        ROS_ERROR("Failed to load camera calibration file.");
        return 1;
    }
    camera_info_msg = cam_info_manager.getCameraInfo();

    // Publish camera info
    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        camera_info_msg.header.stamp = ros::Time::now();
        camera_info_pub.publish(camera_info_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
