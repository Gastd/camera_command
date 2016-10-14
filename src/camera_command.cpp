#include <iostream>
#include "camera_command/camera_command.h"

CameraCommand::CameraCommand(ros::NodeHandle n): nh_(n), pnh_("~"), it_(n)
{
    // Init params
    ros::NodeHandle camera_command_handle(nh_, "camera_command");
    pnh_.param("topic_name", topic_name_, std::string("camera/image"));
    pnh_.param("vel_max_x", vel_max_x_, 0.25);
    pnh_.param("vel_max_yaw", vel_max_yaw_, 0.25);
    pnh_.param("acc", acc_, 0.25);
    pnh_.param("max_distance", max_distance_, 1.);
    pnh_.param("theta", theta_, M_PI/2.); // degrees

    // Init variables
    bgr_mean_.push_back(0); // mean blue
    bgr_mean_.push_back(0); // mean green
    bgr_mean_.push_back(0); // mean red
    init_time_ = ros::Time::now();
    moving_ = false;
    first_stop_ = 0.;
    second_stop_ = 0.;
    third_stop_ = 0.;
    command_ = -1;

    // Init ros comunication
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    command_sub_ = it_.subscribe(topic_name_, 1, &CameraCommand::imageCallback, this);
}

void CameraCommand::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    // std::vector<cv::Mat> BGR(3);

    cv::Scalar sum = cv::Scalar(0, 0, 0);// cv::split(img, BGR);
    // ROS_INFO_STREAM("IMG EMPTY " << img.empty() << " rows " << img.rows << " cols " << img.cols);

    double sum_i = 0, sum_b = 0, sum_g = 0, sum_r = 0;
    for(int i = 0; i < img.rows; ++i)
    {
        for(int j = 0; j < img.cols; ++j)
        {
            for (int k = 0; k < img.channels(); ++k)
            {
                sum_i += img.at<cv::Vec3b>(i,j)[k];
            }
            sum_b += img.at<cv::Vec3b>(i,j)[0];
            sum_g += img.at<cv::Vec3b>(i,j)[1];
            sum_r += img.at<cv::Vec3b>(i,j)[2];
        }
    }
    // mean_ = sum_i/(img.cols*img.rows*img.channels());
    bgr_mean_.at(0) = (sum_b/(img.cols*img.rows));
    bgr_mean_.at(1) = (sum_g/(img.cols*img.rows));
    bgr_mean_.at(2) = (sum_r/(img.cols*img.rows));
    setNewCommand();
    calculateVelocities();
}

void CameraCommand::setNewCommand()
{
    if(moving_)
        return;

    if( bgr_mean_.at(0) >= 170 &&
        bgr_mean_.at(1) <= 80  &&
        bgr_mean_.at(2) <= 80 )
    {
        init_time_ = ros::Time::now();
        command_ = TURN_LEFT;
    }

    else if(bgr_mean_.at(0) <= 80  &&
            bgr_mean_.at(1) >= 170 &&
            bgr_mean_.at(2) <= 80 )
    {
        init_time_ = ros::Time::now();
        command_ = STRAIGHT;
    }

    else if((bgr_mean_.at(0) <= 80 ) &&
            (bgr_mean_.at(1) <= 80 ) &&
            (bgr_mean_.at(2) >= 170))
    {
        init_time_ = ros::Time::now();
        command_ = TURN_RIGHT;
    }
}

void CameraCommand::calculateVelocities()
{
    static int last_comand = -1;
    // static ros::Duration old_time(0.00001);
    if(moving_ && (last_comand == command_))
        return;

    switch(command_)
    {
        case STRAIGHT:  velocityTime(vel_max_x_);
                        break;
        case TURN_LEFT: velocityTime(vel_max_yaw_, false);
                        break;
        case TURN_RIGHT: velocityTime(vel_max_yaw_, false);
                         break;
    }
}

void CameraCommand::velocityTime(double vel_max, bool linear)
{
    first_stop_ = second_stop_ = third_stop_ = 0.;
    first_stop_ = vel_max / acc_;
    double deltaS = acc_ * first_stop_*first_stop_; /* deltaS = 2 * 0.5 * first_stop_^2 */
    if(linear)
    {

        ROS_INFO_STREAM("first_stop_ " << first_stop_);
        ROS_INFO_STREAM("max_distance_ " << max_distance_ << " deltaS " << deltaS);
        if(deltaS > max_distance_)
        {
            ROS_ERROR("Impossible reach the max distance with this acceleration");
            ROS_BREAK();
        }
        else /*if(deltaS <= max_distance_)*/
        {
            second_stop_ = (max_distance_ - deltaS)/vel_max;
            // second_stop_ += first_stop_;
            third_stop_ = second_stop_ + first_stop_;
        }
    }
    else
    {
        ROS_INFO_STREAM("first_stop_ " << first_stop_);
        ROS_INFO_STREAM("theta_ " << theta_ << " deltaS " << deltaS);
        if(deltaS > theta_)
        {
            ROS_ERROR("Impossible reach the max distance with this acceleration");
            ROS_BREAK();
        }
        else /*if(deltaS <= theta_)*/
        {
            second_stop_ = (theta_ - deltaS)/vel_max;
            // second_stop_ += first_stop_;
            third_stop_ = second_stop_ + first_stop_;
        }
    }
}

void CameraCommand::publishCmdVel()
{
    ros::Time time_now = ros::Time::now();
    ros::Duration dt = time_now - init_time_;
    switch(command_)
    {
        case STRAIGHT:  twist_msg_.linear.x = velocity(dt.toSec(), vel_max_x_);
                        moving_ = true;
                        break;
        case TURN_LEFT: twist_msg_.angular.z = velocity(dt.toSec(), vel_max_yaw_);
                        twist_msg_.linear.x = velocity(dt.toSec(), vel_max_x_);
                        moving_ = true;
                        break;
        case TURN_RIGHT:twist_msg_.angular.z = -velocity(dt.toSec(), vel_max_yaw_);
                        twist_msg_.linear.x = velocity(dt.toSec(), vel_max_x_);
                        moving_ = true;
                        break;
        case -1:
                        twist_msg_.linear.x = 0.;
                        twist_msg_.angular.z = 0.;
                        moving_ = false;
                        break;
    }
    if(dt.toSec() > third_stop_)
        moving_ = false;
    ROS_INFO_STREAM("twist_msg_.linear.x " << twist_msg_.linear.x);
    ROS_INFO_STREAM("twist_msg_.angular.z " << twist_msg_.angular.z);
    vel_pub_.publish(twist_msg_);
}

double CameraCommand::velocity(double time, double vel_max)
{
    ROS_INFO_STREAM("time " << time);
    ROS_INFO_STREAM("first_stop_ " << first_stop_);
    ROS_INFO_STREAM("second_stop_ " << second_stop_);
    ROS_INFO_STREAM("third_stop_ " << third_stop_);
    if(time <= first_stop_)
        return (acc_*time);
    else if((time <= second_stop_))
        return vel_max;
    else if(time <= third_stop_)
        return vel_max - (acc_*(time - third_stop_));
    else
        return 0.0;
}

void CameraCommand::spin()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_STREAM("the mean b is " << bgr_mean_.at(0));
        ROS_INFO_STREAM("the mean g is " << bgr_mean_.at(1));
        ROS_INFO_STREAM("the mean r is " << bgr_mean_.at(2));
        publishCmdVel();
        loop_rate.sleep();
    }
}

CameraCommand::~CameraCommand()
{
    /* no code */
}
