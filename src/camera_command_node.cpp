#include "camera_command/camera_command.h"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command");
    ros::NodeHandle n;
    CameraCommand cam_command(n);
    cam_command.spin();
}
