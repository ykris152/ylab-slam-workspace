# 概要
ROSで北陽電機社のURGセンサを使うためのlaunchファイル集  
Top-URG(UTM-30LX)と3D-URG(YVT-X002)のlaunchファイルを用意した  
* Top-URGは[urg_node](http://wiki.ros.org/urg_node)  
* 3D-URGは[hokuyo3d](http://wiki.ros.org/hokuyo3d)  

を使って動作させる

# テスト環境
* UTM-30LX
* YVT-X002
* Let's note CF-SX4
* LinuxMint 18.1 Serena KDE (ubuntu 16.04 LTS)
* ROS Kinetic

# 依存パッケージ
* urg_node
* hokuyo3d
* tf
* rviz

# 使い方 (使用例)
```
$ roscore
$ roslaunch urg_launch urg.launch
```
2d.launch: Top-URGだけを動かす  
3d.launch: 3D URGだけを動かす  
urg.launch: Top-URGと3D URGの両方を動かす  