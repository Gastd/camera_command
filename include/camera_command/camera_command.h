#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class CameraCommand
{
public:
    CameraCommand(ros::NodeHandle nh);
    void spin();
    ~CameraCommand();

    enum MOVE
    {
        TURN_LEFT,
        STRAIGHT,
        TURN_RIGHT,
    };

private:
    int command_;
    bool moving_;
    std::vector<int> bgr_mean_;
    double vel_max_x_, vel_max_yaw_, acc_, max_distance_, theta_;
    double first_stop_, second_stop_, third_stop_;

    std::string topic_name_;
    ros::NodeHandle nh_, pnh_;
    ros::Publisher vel_pub_;
    ros::Time init_time_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber command_sub_;
    geometry_msgs::Twist twist_msg_; 

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void calculateVelocities();
    void velocityTime(double vel_max, bool linear = true);
    double velocity(double time, double vel_max);
    void publishCmdVel();
    void setNewCommand();
};
