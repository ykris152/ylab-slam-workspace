# 概要
ROSでi-Cart miniを動かすためのlaunchファイル集  
i-Cart用のドライバは[ypspur_ros](https://github.com/openspur/ypspur_ros)  
i-Cartをゲームパッドで動かすには[teleop_twist_joy](http://wiki.ros.org/teleop_twist_joy)  
を使用する

# テスト環境
* i-Cart mini
* Xbox One Controller
* Let's note CF-SX4
* LinuxMint 18.1 Serena KDE (ubuntu 16.04 LTS)
* ROS Kinetic

# 依存パッケージ
* ypspur_ros
* teleop_twist_joy
* joy  
だけのはず

# 使い方 (使用例)
```
$ roscore
$ roslaunch ypspur_launch ypspur.launch
$ roslaunch ypspur_launch teleop.launch
```

## 箱1の設定ファイルについて
EnableボタンかTurboボタンを押しながらでないと動かない  
teleop_twist_joyのデフォルトでは片手で操作できるようになっていたが, 安全面を考えると両手でしっかり操作できるようにすべきだと判断した  
Enable, Turboボタンには軸(トリガーとかスティック)が設定できないようだったので、LRバンパーで代用

|Part|Button or Axis|
|:--:|:--:|
|Enable|Left Bumper|
|Turbo|Right Bumper|
|Move forward|Left stick up|
|Move backward|Left stick down|
|Right turn|Right stick right|
|Left turn|Right stick left|

## DS3の設定ファイルについて
|Part|Button or Axis|
|:--:|:--:|
|Enable|L2 Trigger|
|Turbo|L1 Button|
|Move forward|Left stick up|
|Move backward|Left stick down|
|Right turn|Right stick right|
|Left turn|Right stick left|