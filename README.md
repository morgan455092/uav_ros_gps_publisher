# uav_ros_gps_publisher

## 簡介
本專案是一個ROS節點，負責模擬或讀取GPS座標數據並將其發布到ROS的`/gps`主題。該節點使用C++編寫，適用於ROS Noetic環境。您可以透過該節點模擬GPS數據或與真實的GPS硬體整合，用於導航、機器人定位或其他應用。

## 功能
- 模擬GPS數據 (經度與緯度)
- 將GPS座標以5 Hz的頻率發布到`/gps`主題
- 易於整合實際硬體數據源

## 系統需求
- **ROS版本**: Noetic
- **作業系統**: Ubuntu 20.04
- **編譯工具**: CMake, GCC
- **依賴套件**:
  - `roscpp`
  - `std_msgs`

## 建置方式

### 1. 建立`catkin`工作空間
如果尚未建立工作空間，請執行以下命令：
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

### 2. 克隆或建立套件
進入工作空間的`src`目錄並建立新套件：
```bash
cd ~/catkin_ws/src
catkin_create_pkg gps_publisher roscpp std_msgs
```

將`gps_publisher.cpp`程式碼放置於`src`資料夾中，檔案路徑應為：
```
~/catkin_ws/src/gps_publisher/src/gps_publisher.cpp
```

### 3. 修改`CMakeLists.txt`
在`gps_publisher`套件目錄內修改`CMakeLists.txt`，添加以下內容：

#### 添加可執行檔案與連結庫：
```cmake
add_executable(gps_publisher src/gps_publisher.cpp)
target_link_libraries(gps_publisher ${catkin_LIBRARIES})
```

#### 確保依賴項目如下：
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
```

### 4. 編譯專案
回到工作空間根目錄，執行以下命令：
```bash
cd ~/catkin_ws
catkin_make
```

## 使用方式

### 啟動ROS主控程式
在終端啟動ROS主控程式：
```bash
roscore
```

### 啟動`gps_publisher`節點
在新的終端運行以下命令啟動節點：
```bash
rosrun gps_publisher gps_publisher
```

### 查看GPS數據
在另一個終端中，使用以下命令訂閱`/gps`主題查看GPS座標：
```bash
rostopic echo /gps
```

您將看到類似以下的輸出：
```
data: "Latitude: 37.7750, Longitude: -122.4195"
---
data: "Latitude: 37.7751, Longitude: -122.4196"
---
```

## 程式架構與邏輯
1. **模擬GPS數據**:
   預設使用一個簡單的函數模擬經緯度數據 (`getGPSData`)。
2. **頻率控制**:
   使用`ros::Rate`控制節點發布的頻率 (5 Hz)。
3. **主題發布**:
   GPS數據以字串格式發布到`/gps`主題，可供其他ROS節點訂閱使用。


## 注意事項
1. **實際硬體整合**:
   若需整合實際GPS硬體，請替換`getGPSData`函數的實現，將其改為讀取GPS裝置的數據。
2. **頻率調整**:
   可修改程式中的`ros::Rate loop_rate(5);`以調整數據發布頻率。
3. **測試環境**:
   該程式已在Ubuntu 20.04與ROS Noetic環境下測試通過。
