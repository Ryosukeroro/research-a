/**
* @file utilities.h
*/

#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>

// rad⇔deg変換
#define deg_to_rad(deg) (((deg)/360)*2*M_PI)
#define rad_to_deg(rad) (((rad)/2/M_PI)*360)

// ROS Time
#define ROS_NON_RATE 0.0

// queueサイズ
#define ROS_QUEUE_SIZE_1 1


template <typename T>
bool getParam(const ros::NodeHandle &node, const std::string get_key, T &param, T def_param)
{
    bool is_success = false;

    if(node.getParam(get_key, param))
    { // 読み込み成功
        ROS_INFO_STREAM("Read success param key: " << get_key.c_str() << " / value: "<< param);
        is_success = true;
    }
    else
    { // 読み込み失敗
        ROS_WARN_STREAM("Read failure param key: " << get_key.c_str() << " / value: "<< def_param);
        param = def_param;
    }

    return (is_success);
}

std::string rosTimeToIso8601(ros::Time time);

bool strTomeToRosTime(ros::Time &res_time, std::string data_time, std::string format);

std::string iso8601ex(void);

bool checkLastPublishTime(double freq, ros::Time last_pub_time);

#endif
