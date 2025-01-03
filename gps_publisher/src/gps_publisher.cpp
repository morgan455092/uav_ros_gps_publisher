#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>

// 模擬 GPS 座標生成
std::tuple<double, double, double> getGPSData() {
    // 實際應用中，可從硬體或文件讀取座標
    static double latitude = 37.7749;   // 模擬緯度 (可替換為實際數據)
    static double longitude = -122.4194; // 模擬經度 (可替換為實際數據)
    static double altitude = 10.0;       // 模擬海拔高度 (可替換為實際數據)
    
    return std::make_tuple(latitude, longitude, altitude);
}

int main(int argc, char **argv) {
    // 初始化節點
    ros::init(argc, argv, "gps_publisher");

    // 創建 NodeHandle
    ros::NodeHandle n;

    // 創建 Publisher，發布到主題 /gps
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("/gps", 10);

    // 設置發布頻率 (5 Hz)
    ros::Rate loop_rate(5);

    while (ros::ok()) {
        // 獲取 GPS 數據
        auto [latitude, longitude, altitude] = getGPSData();

        // 構建 NavSatFix 消息
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "gps_frame"; // 根據需要設置框架 ID
        msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; // 假設已獲得定位
        msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // 使用 GPS 服務
        msg.latitude = latitude;
        msg.longitude = longitude;
        msg.altitude = altitude;

        // 設置位置協方差 (此處假設未知)
        for (double &cov : msg.position_covariance) {
            cov = 0.0;
        }
        msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        // 打印資訊到控制台
        ROS_INFO("Latitude: %f, Longitude: %f, Altitude: %f", latitude, longitude, altitude);

        // 發布消息
        gps_pub.publish(msg);

        // 確保頻率為 5 Hz
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
