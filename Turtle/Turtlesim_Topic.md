# Turtlesim Topic
Reference: https://cafe.naver.com/openrt/24101 <br/>

## 1.Run Turtlesim Node
```
ros2 run turtlesim turtlesim_node
```

## 2.Topic Commands

### list
현재 실행 중인 노드들의 서비스 목록 <br/>
```
ros2 service list
```
다양한 서비스들이 포함되어 있는데 parameters 가 붙어있는 서비스는 파라미터와 관련된 내용으로 모든 노드의 기본 기능으로 포함되어 있다 <br/>

### info
```
ros2 node info /turtlesim
```
-`ros2 node info` 명령어를 이용하여 turtlesim_node (노드명: turtlesim, 이하 turtlesim이라 표기함) 노드의 토픽 정보를 확인 <br/>
-참고로 turtlesim_node 노드는 turtlesim이라는 노드 이름으로 실행된다. 서비스, 액션, 파라미터를 제거하여 본다면 아래와 같을 것이다. <br/>
turtlesim 노드는 geometry_msgs/msg/Twist 형태의 메시지인 cmd_vel을 구독하고 있다는 것과 turtlesim/msg/Color 형태의 color_sensor 메시지 형태인 color_sensor, 그리고 turtlesim/msg/Pose 형태의 pose 메시지를 발행하고 있다는 것이다 <br/>

### list_-t
```
ros2 topic list -t
```
좀 더 간단하게 메시지들을 확인 <br/>
turtlesim 노드만의 정보를 확인하는 것이 아니라 현재 개발 환경에서 동작 중인 모든 노드들의 토픽 정보를 볼 수 있는 것으로 지금은 turtlesim 노드만이 실행된 상태이기에 turtlesim 노드가 발행, 구독하는 메시지만 표시되고 있다. <br/>
참고로 `-t` 옵션은 부가적인 것으로 각 메시지의 형태(type)를 함께 표시해준다 <br/>

### rqt_graph
```
rqt_graph
```

### topic_echo
```
ros2 topic echo /turtle1/cmd_vel
```
-특정 토픽의 메시지 내용을 실시간으로 표시하는 `ros2 topic echo`를 사용해보자 <br/>
다음 명령어와 같이 `/turtle1/cmd_vel`라고 토픽을 지정하게 되면 해당 토픽의 값을 확인해볼 수 있다 <br/>
-참고로 `/turtle1/cmd_vel` 토픽을 발행하는 teleop_turtle 노드를 실행한 터미널 창에서 방향 키보드 키(←↑ ↓→)를 눌러 명령을 내려야지만 토픽 값을 확인할 수 있다 <br/>
아래의 결과를 보자면 `/turtle1/cmd_vel` 토픽의 linear에 x, y, z 값이 있으며, angular에 x, y, z 값이 존재한다는 것을 알 수 있다 <br/>
총 6개의 값으로 구성되어 있으며 현재 linear.x 값으로 1.0 m/s 임을 확인할 수 있다 <br/>
-참고로 모든 메시지는 meter, second, degree, kg 등 SI 단위를 기본으로 사용한다 <br/>
 <br/>
### topic_bw
```
ros2 topic bw /turtle1/cmd_vel
```
메시지의 대역폭, 즉 송수신받는 토픽 메시지의 크기를 확인해보자 <br/>
크기 확인은 아래 명령어와 같이 `ros2 topic bw`으로 지정된 토픽 메시지의 송수신되는 토픽의 초당 대역폭을 알 수 있다 <br/>
teleop_turtle 노드에서 지속적으로 메시지를 보내는 상황이라면 평균 1.74KB/s의 대역폭으로 /turtle1/cmd_vel 토픽이 사용되는 것을 확인할 수 있다 <br/>
이는 사용하는 메시지 형태 및 주기에 따라 달라질 수 있다 <br/>

### topic_hz
토픽의 전송 주기를 확인하려면 `ros2 topic hz` 명령어를 이용하면 된다 <br/>
```
ros2 topic hz /turtle1/cmd_vel
```
teleop_turtle 노드에서 지속적으로 /turtle1/cmd_vel 토픽을 발행한다면 아래와 같이 평균 33.2 Hz 정도가 나올 것이다 <br/>
즉 0.03 초에 한번씩 토픽을 발행하고 있다는 것이다 <br/>
이는 teleop_turtle 노드에서 얼마나 자주 /turtle1/cmd_vel 토픽을 발행하는지에 따라 달라질 수 있다 <br/>

### topic delay
```
ros2 topic delay /TOPIC_NAME
```
토픽은 RMW 및 네트워크 장비를 거치기 때문에 latency 즉 지연 시간이 반드시 존재하게 된다 <br/>
이 지연 시간을 체크하는 방식으로 유저가 직접 코드로 구현하는 방법도 있겠지만 <br/>
메시지내에 header[6]라는 stamp 메시지를 사용하고 있다면 `ros2 topic delay`를 명령어를 이용하여 메시지는 발행한 시간과 구독한 시간의 차를 계산하여 지연 시간을 확인할 수 있다 <br/>
### topic pub
```
ros2 topic pub <topic_name> <msg_type> "<args>"
```
-토픽의 발행(publish)은 ROS 프로그램에 내장하는게 기본이다 <br/>
이는 ROS 프로그래밍 시간에 다루도록 하고 여기서는 `ros2 topic pub` 명령어를 통해 간단히 토픽을 발행하는 테스트를 해보자 <br/>
-`ros2 topic pub` 명령어에 토픽 이름, 토픽 메시지 타입, 메시지 내용을 기술하면 된다. <br/>
<br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
--once` 옵션을 사용하여 단 한번의 발행만을 수행함 <br/>
토픽 이름으로는 /turtle1/cmd_vel 을 사용하였고, 토픽 메시지 타입은 geometry_msgs/msg/Twist 을 사용 <br/>
메시지 내용으로는 병진 속도 linear.x 값으로 2.0 m/s를 넣었고, 회전 속도 angular.z 값으로 1.8 rad/s를 입력하였다 <br/>

**pub_forward** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
**pub_backward** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
**pub_half_a_circle_left** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.14}}" # 반지름 4.0
```
**pub_half_a_circle_right** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -3.14}}" # 반지름 4.0
```
**pub_circle_right** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"
```
**pub_circle_left** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}" 
```
**pub_idle_circle_left** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"
```

**pub_idle_circle_right** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}"
```

**pub_turnleft_90degrees** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.57}}"  # Turn left 90 degrees (π/2 radians)
```
**pub_turnright_90degrees** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.57}}"  # Turn left 90 degrees (π/2 radians)
```
**pub_turnleft_180degrees** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.14}}"  # Turn left 180 degrees (π radians)
```
**pub_turnright_180degrees** <br/>
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -3.14}}"  # Turn left 180 degrees (π radians)
```


### bag record
```
ros2 bag record <topic_name1> <topic_name2> <topic_name3>

ros2 bag record /turtle1/cmd_vel
```

### bag info
```
ros2 bag info rosbag2_2024_12_18-21_16_29/
```
저장된 rosbag 파일의 정보를 확인하려면 아래 예제와 같이 bag 정보 (ros2 bag info) 명령어를 이용하면 된다 <br/>
내용을 살펴보면 방금 전 우리가 기록한 이 rosbag 파일은 84.4 KiB 크기에 31.602s 시간 동안 기록되었고 <br/>
기록이 언제 시작되고 언제 끝났는지 타임스태프와 취득한 토픽의 이름 메시지 형태 메시지 별 갯수와 총 갯수 등이 기록되어 있다 <br/>


### bag play
```
ros2 run turtlesim turtlesim_node

ros2 bag play rosbag2_2024_12_18-21_16_29/
```
rosbag 파일을 기록하고 정보를 확인해봤으니 이제는 재생을 해보자 <br/>
일단 turtlesim 노드를 종료한 후 다시 시작하여 초기화를 해준 후 아래의 예제처럼 rosbag를 재생하면 기록 시간 타이밍에 따라 토픽이 재생됨을 확인할 수 있다 <br/>
이는 위에서 설명한 `ros2 topic echo /turtle1/cmd_vel` 명령어를 이용하여 터미널 창에서 학인해도 되고, 그림 11와 같이 turtlesim 노드위의 거북이의 움직임을 비교해도 된다 <br/>

### message interface
```
ros2 interface show geometry_msgs/msg/Twist

ros2 interface show geometry_msgs/msg/Vector3
```
