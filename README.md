# 足立くん用の研究サポート
保田先生に面倒見てと言われてるので、これくらいは...

# 実験のための準備の流れ
シミュレータを起動

↓

gmapping起動

↓

地図作成のためにキーボード操作起動

↓

地図作成を行う(ロボットを動かす)

↓

地図を保存

# 実験のための準備
以下はロボットモデルの選択です。
```bash
export TURTLEBOT3_MODEL=burger
```
次はGazeboシミュレータの起動です。
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
次はgmappingの起動です。
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
次はロボットを動かすために起動
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
一通り動かして地図が作成できたら、以下のコマンドを使って地図データを保存してください。
これで実験準備は完了です。
```
rosrun map_server map_saver -f ~/map
```


# 実験
再びシミュレーションを起動します。
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Navigationを起動します。ファイルの場所はHOMEにしていますが、ご自分で合わせてください。
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
初期位置が合わないので、AMCLではロボットの初期位置は(0,0),turtlebotは(2,-0.5)なので、AMCLの推定位置を移動させます。
```
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position: {x: -2.0, y: -0.5, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]"
  ```
これであとはNaviGoalを指定するだけでロボットが自律移動します。

...
残念ながら、この段階では研究とは言えません。ここまでしても、何の新規性もないからです。

新しい手法の提案や既存手法の新しい使い方、改良、難しい組み合わせを行ってこそ、この分野の研究でしょう。

引き継ぎとして、まずは既存のICPを実装してみます。

SLAMは難しいのでB4は自己位置推定で良いと思います。度胸があるならSLAMしましょう。

まず、前提として、自己位置推定には地図データが必要です。

実験準備の段階で作った地図は占有格子地図です。

この生データをそのまま使ってもICPはできません。

生データを座標データに変換してあげる必要があります。

まずはそこから行いましょう！
