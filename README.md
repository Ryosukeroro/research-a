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
