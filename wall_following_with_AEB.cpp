#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "math.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#define KP 1.00
#define KD 0.001  
#define KI 0.005
#define DESIRED_DISTANCE_RIGHT 1.0  //AB
#define DESIRED_DISTANCE_LEFT 1.0
#define LOOK_AHEAD_DIS 1.0
#define PI 3.1415927

class SubscribeAndPublish1 {
public:
    SubscribeAndPublish1() {
        //1、发布与订阅的话题
        drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1000);
        scan_sub = nh.subscribe("/scan", 1000, &SubscribeAndPublish1::callback, this);
    }

    void callback(const sensor_msgs::LaserScan& lidar_info) {

        //2、获取激光雷达测量距离 getRange
        unsigned int b_index = (unsigned int)(floor((90.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));  //dist = data.ranges[int(index)]
        double b_angle = 90.0 / 180.0 * PI;    //两条射线之间的角度   
        double a_angle = 45.0 / 180.0 * PI; 
        unsigned int a_index;
        if (lidar_info.angle_min > 45.0 / 180.0 * PI) {
            a_angle = lidar_info.angle_min;
            a_index = 0;
        } else {
            a_index = (unsigned int)(floor((45.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
        }            
        double a_range = 0.0;   
        double b_range = 0.0;
        if (!std::isinf(lidar_info.ranges[a_index]) && !std::isnan(lidar_info.ranges[a_index])) {   //isinf()是否为无穷大   ranges：转一周的测量数据一共360个，每度是一个
            a_range = lidar_info.ranges[a_index];   //得到a的长度
        } else {
            a_range = 100.0;
        }
        if (!std::isinf(lidar_info.ranges[b_index]) && !std::isnan(lidar_info.ranges[b_index])) {
            b_range = lidar_info.ranges[b_index];   //得到b的长度
        } else {
            b_range = 100.0;
        }
        //3、计算公式，参考pdf  
        //在车的右边得到两条射线a、b来确定车到右墙的距离AB和相对于AB的方向
        double alpha = atan((a_range * cos(b_angle - a_angle) - b_range) / (a_range * sin(b_angle - a_angle)));   //b_angle - a_angle为a、b夹角THETA
        double AB = b_range * cos(alpha);    //实际离右墙距离
        double projected_dis = AB + LOOK_AHEAD_DIS * sin(alpha);     //Due to the forward motion of the car and a finite delay in execution of the control and perception nodes; 
                                                                     //we instead, virtually project the car forward a certain distance from its current position.  
                                                                     //LOOK_AHEAD_DIS = AC = 1   projected_dis = CD
        error = DESIRED_DISTANCE_LEFT - projected_dis;   //求出误差
        // ROS_INFO("projected_dis = %f",projected_dis);
        // ROS_INFO("error = %f",error);
        // ROS_INFO("del_time = %f\n",del_time);
        ROS_INFO("alpha = %f\n",180 * alpha / PI);
        SubscribeAndPublish1::pid_control();
    }

        //4、PID控制器
    void pid_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        double tmoment = ros::Time::now().toSec();
        del_time = tmoment - prev_tmoment;   //当前时刻-上一个时刻 = 间隔时刻
        integral += prev_error * del_time;   //对误差积分，也就是误差的无限和  积分=积分+累计误差
        ackermann_drive_result.drive.steering_angle = -(KP * error + KD * (error - prev_error) / del_time + KI * integral);
        prev_tmoment = tmoment;    //时间的迭代
        //不同情况下的速度调整，转弯时速度降低，直行时速度加快
        if (abs(ackermann_drive_result.drive.steering_angle) > 40.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 1.0;
        } else if (abs(ackermann_drive_result.drive.steering_angle) > 30.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 2.0;
        } else if (abs(ackermann_drive_result.drive.steering_angle) > 20.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 3.0;
        } else if (abs(ackermann_drive_result.drive.steering_angle) > 10.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 4.0;
        } else if (abs(ackermann_drive_result.drive.steering_angle) > 1.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 5.0;
        } else {
            ackermann_drive_result.drive.speed = 6.0;
        }
        drive_pub.publish(ackermann_drive_result);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher drive_pub;
    ros::Subscriber scan_sub;
    double prev_error = 0.0;    //前一个误差
    double prev_tmoment = ros::Time::now().toSec();    //当前时间，单位秒s
    double error = 0.0;
    double integral = 0.0;
    double speed = 0.0;
    double del_time = 0.0;

};

class SubscribeAndPublish2 {
public:
    enum class WeatherCondition {
        Dry,
        Wet,
        Snow,
        // Add more weather conditions if needed
    };

    WeatherCondition getCurrentWeather() {
        // it is a simple demo. We can get the current weather condition from other sensors!
        return WeatherCondition::Dry;
    }
    SubscribeAndPublish2() {
        boolPub = n.advertise<std_msgs::Bool>("/brake_bool", 1000);//bool信息，表示汽车是否开启了AEB
        ackermannPub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);//向模拟器和汽车发送刹车命令的消息类型
        scanSub = n.subscribe("/scan", 1000, &SubscribeAndPublish2::callback_laserscan, this);//订阅scan topic并用LaserScan信息计算TTC
        odomSub = n.subscribe("/odom", 1000, &SubscribeAndPublish2::callback_odometry, this);//模拟器节点和CAR本身都发布Odometry消息，包括汽车的位置、方向和速度
    }

    void callback_laserscan(const sensor_msgs::LaserScan& lidar_info) {
        lidar_info_local = lidar_info;
    }

    void callback_odometry(const nav_msgs::Odometry& odometry_info) {
        odometry_info_local = odometry_info;
    }

    void publish() {
        double TTC_threshold = 0.4;//调参的结果
        double min_TTC = 100;//设置一个很大的大于阈值的初始值
        double v_x = odometry_info_local.twist.twist.linear.x;
        double v_y = odometry_info_local.twist.twist.linear.y;
        double forward_length = 0.0;
        double backward_length = 0.0;
        WeatherCondition weather = getCurrentWeather();
        double friction_coefficient = 0.0;
        switch (weather) {
            case WeatherCondition::Dry:
                friction_coefficient = 0.8;  // Dry road friction coefficient
                break;
            case WeatherCondition::Wet:
                friction_coefficient = 0.4;  // Wet road friction coefficient
                break;
            case WeatherCondition::Snow:
                friction_coefficient = 0.2;  // Snowy road friction coefficient
                break;
            default:
                friction_coefficient = 0.6;  // Default friction coefficient
                break;
        }

        // Calculate the distance based on vehicle speed and friction coefficient
        double TTC_distance = (v_x * v_x + v_y * v_y) / (2.0 * friction_coefficient * 10);

        // Check if lidar information is available[add]
        if (lidar_info_local.ranges.empty()) {
            ROS_WARN("No laser scan data available.");
            return;
        }
        //计算激光扫描中每束光束的TTC，取最小值与阈值对比，决定是否开启AEB
        for (unsigned int i = 0; i < lidar_info_local.ranges.size(); i++) {
            if (!std::isinf(lidar_info_local.ranges[i]) && !std::isnan(lidar_info_local.ranges[i])) {
                double distance = lidar_info_local.ranges[i];//实际距离：r=distance
                //得到Cars速度向量和距离向量之间的角度 angle_min：可检测范围的起始角度 angle_increment：采集到相邻数据帧之间的角度步长
                double angle = lidar_info_local.angle_min + lidar_info_local.angle_increment * i;
                double distance_derivative = cos(angle) * v_x + sin(angle) * v_y;//分母：实际距离的时间导数
                //分母是正数时才有比较的意义，否则分母为0，TTC为正无穷
                if (distance_derivative > 0 && distance / distance_derivative < min_TTC) min_TTC = distance / distance_derivative;
            }
        }
        if(v_x > 0){
            // forward
            forward_length = TTC_distance / sqrt(v_x * v_x + v_y * v_y) + 0.2;
            TTC_threshold = forward_length;
            // ROS_INFO("forward");        
        }else{
            backward_length = TTC_distance / sqrt(v_x * v_x + v_y * v_y) + 0.3;
            TTC_threshold = backward_length;
            // ROS_INFO("backward");        
        }
        if(v_x * v_x + v_y * v_y > 50){
        // we should give more time to react when the car is running at high speed!
            TTC_threshold *= 1.5;
        }
    if (min_TTC <= TTC_threshold) {
        std_msgs::Bool brake_bool_result;
        brake_bool_result.data = true;
        boolPub.publish(brake_bool_result);

        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        // for test
        // v_x = 6;v_y = 6;
         if (v_x * v_x + v_y * v_y > 50) {
        // Keep the speed decrease if the vehicle speed is greater than 7m/s. it is not safe to speed down suddenly!
        ackermann_drive_result.drive.speed = sqrt(v_x * v_x + v_y * v_y) / 4;
        ackermannPub.publish(ackermann_drive_result);
        ROS_INFO("speed down!");
        // since we have little time to react, after the speed down, we must give the prompt to bring the vehicle to a complete stop at the same time!
        ackermann_drive_result.drive.speed = 0.0;
            } else {
        // Set the speed to 0 if the vehicle speed is less than or equal to 7
        ackermann_drive_result.drive.speed = 0.0;
    }
        // ackermann_drive_result.drive.speed = 0.0;
        ackermannPub.publish(ackermann_drive_result);
        ROS_INFO("Emergency brake engaged");
    }
    else {
        std_msgs::Bool brake_bool_result;
        brake_bool_result.data = false;
        boolPub.publish(brake_bool_result);
    }
    }

private:
    ros::NodeHandle n;
    ros::Publisher boolPub;
    ros::Publisher ackermannPub;
    ros::Subscriber scanSub;
    ros::Subscriber odomSub;
    sensor_msgs::LaserScan lidar_info_local;//把lidar信息存储在本地
    nav_msgs::Odometry odometry_info_local;//把odometry信息存储在本地
};  // End of class SubscribeAndPublish



int main(int argc, char** argv) {

    ros::init(argc, argv, "wall_following_pid");
    SubscribeAndPublish1 SAPObject1;
    ros::spin();
    //Initiate ROS
    ros::init(argc, argv, "safety_node");
    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish2 SAPObject2;//创建类的实例
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        SAPObject2.publish();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
