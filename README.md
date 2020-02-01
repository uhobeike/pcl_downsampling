## 動作環境

以下の環境にて動作確認を行っています。


- ROS Melodic
- OS: Ubuntu 18.04.3 LTS
- ROS Distribution: Melodic Morenia 1.14.3
- Rviz 1.12.16
- PCL 1.8.1
## インストール方法
以下の方法でPCL_studyというパッケージをクローンします。
 ```bash
 cd ~/catkin_ws/src && git clone https://github.com/uhobeike/PCL_study.git
 ```
 以上のコマンドよりPCL_studyというパッケージをクローンします。
`catkin_make`を使用して本パッケージをビルドします。
 ```bash
  cd ~/catkin_ws && catkin_make
  ```
  無事コンパイル通ればOKです。
# PCL_study

`realsensD435i` を使って点群処理をし、物体検出を行うためのパッケージです。


| 使用するパッケージ内プログラム名(PCL_study) | 機能説明 |
----|----
| model_plane_cut_test.cpp | ダウンサンプリングや平面除去やクラスタリングなどの処理を行い物体を検出する |

## システムの起動方法


- まずはじめに、下記のようなコマンドを打ちいらない機能を使用不可にしてpointcloudのトピックを受け取り可能にさせます。
>使用不可にする理由はこのページの一番下にあるブログ（URLを押して）に行くと詳しく書いてあります
```sh
roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true enable_infra2:=false  enable_infra1:=false  enable_gyro:=false enable_stereo:=false
```
- 次にmodel_plane_cut_test.cppを動かすために以下のようなコマンドを実行します。
```sh
rosrun pcl_ros_processing model_plane_cut_test input:=/camera/depth/color/points
```
- 次に点群データの表示方法です。表示を行うには、以下のようなコマンドを打ちrvizを使用します。
```sh
rosrun rviz rviz
```
そうすると以下のような図のようなwindow(rviz設定済み)が開きます。
最初は何も設定していない状態なので、何も表示されていません。そこで、今回のmodel_plane_cut_test.cppを使用する場合はFixedFrameをexample_frameにし
AddボタンからbytopicのPoimtcloud2のmodel_plane_cut_test.cppを指定すると以下のように点群データを表示することができます。
（model_plane_cut_test.cppはダウンサンプリングしてあるので点々が出てきます。）


rvizで表示されている点群データについて
![rviz](https://github.com/ShioriSugiyama/crane_x7_ros/blob/image/image/20191230001833.gif "rviz")


以上のノードより物体検出した際の点群データは以下のようになっています。

物体検知完了時の点群データについて
![crane_x7_e](https://github.com/ShioriSugiyama/crane_x7_ros/blob/image/image/11345882994648.gif "crane_x7_e")




[PCLについて色々探ってブログにメモしたやつ](https://beike.hatenablog.jp/entry/2019/12/24/224303)
