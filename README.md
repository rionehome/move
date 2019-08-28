# move
## Overview
PID制御でkobukiを制御するためのパッケージです。  
速度指定はVelocityノードにtopic通信で、距離＆角度指定はAmountノードでactionを利用します。

## setup
```
git clone https://github.com/rionehome/move.git 
catkin_make
```

## Usage
速度指定
```
rosrun move velocity
```

距離＆角度指定
```
rosrun move velocity
rosrun move amount
```

## Message
**`name` Velocity.msg**
```
float64 linear_rate　#最大速度0.7m/sを１とした直進速度の比率
float64 angular_rate　#最大速度110度/sを１とした回転速度の比率
```
**`name` Amount.msg**
```
float64 distance　#現在の位置から進む距離
float64 angle　#現在の位置から回転する角度
```

**`name` Amount.action**
```
Amount amount　#目標距離＆角度
Velocity velocity　#目標速度
---
bool finished　#到着の成功(True)/失敗(False)
---
move/Amount current_amount　#現在の状態
```

## Node
**`name` velocity**

### Subscribe Topic

* **`/move/velocity`** 速度指定（ move/Velocity ）

**`name` amount**

### Subscribe Topic

* **`/move/amount`** 距離＆角度指定（ move/Amount ）