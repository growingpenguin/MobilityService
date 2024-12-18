# Turtlesim Action
Reference: https://cafe.naver.com/openrt/24142 <br/>

## Run Node
### 1.Run Turtlesim Node
```
ros2 run turtlesim turtlesim_node

ros2 run turtlesim turtle_teleop_key
```
turtlesim 노드와 teleop_turtle 노드를 이용하여 테스트 <br/>
-지금까지는 teleop_turtle 노드가 실행된 터미널 창에서 ← ↑ ↓ → 키와 같이 화살표 키를 눌러 turtlesim의 거북이를 움직였지만 <br/>
이번에는 G, B, V, C, D, E, R, T 키를 사용해보겠다 <br/>
이 키들은 각 거북이들의 rotate_absolute 액션을 수행함에 있어서 액션의 목표 값을 전달하는 목적으로 사용된다 <br/>
-F키를 중심으로 주위의 8개의 버튼을 사용하는 것으로 그림 3와 같이 각 버튼은 거북이를 절대 각도로 회전하도록 목표 값이 설정되어 있다 <br/>
그리고 F 키를 누르면 전달한 목표 값을 취소하여 동작을 바로 멈추게 된다 <br/>
여기서 G 키는 시계 방향 3시를 가르키는 theta 값인 0.0 값에 해당되고 rotate_absolute 액션의 기준 각도가 된다 <br/>
다른 키는 위치별로 0.7854 radian 값씩 정회전 방향(반시계 방향)으로 각 각도 값이 할당되어 있다 <br/>
-예를 들어 R 키를 누르면 1.5708 radian 목표 값이 전달되어 거북이는 12시 방향으로 회전하게 된다. 각 키에 할당된 라디안 값은 [8] 코드를 확인하면 자세히 살펴볼 수 있다. <br/>
-액션 목표는 도중에 취소할 수도 있는데 turtlesim_node 에도 그 상황을 터미널 창에 표시해준다 <br/>
예를 들어 액션 목표의 취소 없이 목표 theta 값에 도달하면 아래와 같이 표시된다 <br/>
[INFO]: Rotation goal completed successfully <br/>
하지만 액션 목표 theta 값에 도달하기 전에 turtle_teleop_key가 실행된 터미널 창에서 F 키를 눌러 액션 목표를 취소하게 되면 turtlesim_node이 실행된 터미널 창에 아래와 같이 목표가 취소 되었음을 알리면서 거북이는 그 자리에서 멈추게된다.
[INFO]: Rotation goal canceled <br/>
<br/>

### 2.Action Commands

**node_info** <br/>
```
ros2 node info /turtlesim

ros2 node info /teleop_turtle
```
다양한 서비스들이 포함되어 있는데 parameters 가 붙어있는 서비스는 파라미터와 관련된 내용으로 모든 노드의 기본 기능으로 포함되어 있다 <br/>

**list_-t** <br/>
```
ros2 action list -t
```
액션 정보를 확인하는 방법 <br/>
위와 같이 특정 노드의 정보를 확인하는 방법 이외에도 액션 목록 (ros2 action list -t) 명령어를 이용하여 현재 개발 환경에서 실행 중인 액션의 목록을 확인하는 방법도 있다 <br/>
<br/>

**action_info** <br/>
```
ros2 action info /turtle1/rotate_absolute
```
**action_goal** <br/>
위에서는 teleop_turtle를 Action client로 하여 액션 목표(action goal)를 전달해보았다 <br/>

```
ros2 action send_goal <action_name> <action_type> "<values>"
```
-이번에는 `ros2 action send_goal` 명령어를 통하여 액션 목표(action goal)를 전달해보겠다 <br/>
이 명령어는 아래와 같이 `ros2 action send_goal` 명령어에 액션 이름, 액션 형태, 목표 값을 차례로 입력하면 된다 <br/>
-그림와 같이 이동하게 되며 아래 처럼 전달한 목표 값과 액션 목표의 UID(Unique ID), 결괏값으로 이동 시작 위치로의 변위 값인 delta를 결과로 보여주며 마지막으로 상태를 표시하게 된다 <br/>
<br/>
**Action_Forward** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5708}"
```
거북이를 12시 방향인 theta: 1.5708 값을 목표로 준다 <br/>
 <br/>
**Action_ForwardRight** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 0.7854}"
```
거북이를 1시 30분 방향인 theta: 0.7854 값을 목표로 준다 <br/>
 <br/>
**Action_Rightward** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 0.0}"
```
거북이를 3시 방향인 theta: 0.0 값을 목표로 준다 <br/>
 <br/>
**Action_BackwardRight** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -0.7854}"
```
거북이를 1시 30분 방향인 theta: 0.7854 값을 목표로 준다 <br/>
<br/>
**Action_Backward** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.5708}"
```
거북이를 6시 방향인 theta: -1.5708 값을 목표로 준다 <br/>
<br/>
**Action_BackwardLeft** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -2.3562}"
```
거북이를 6시 방향인 theta: -2.3562 값을 목표로 준다 <br/>
<br/>
**Action Leftward** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.1416}"
```
거북이를 3시 방향인 theta: 3.1416 값을 목표로 준다 <br/>
<br/>
**Action_ForwardLeft** <br/>
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 2.3562}"
```
거북이를 10시 30분 방향인 theta: 2.3562 값을 목표로 준다 <br/>


**action_interface** <br/>
액션 또한 토픽, 서비스와 마찬가지로 별도의 인터페이스를 가지고 있는데 이를 액션 인터페이스라 부르며, 파일로는 action 파일을 가르킨다 <br/>
액션 인터페이스는 메시지 및 서비스 인터페이스의 확장형이라고 볼 수 있는데 위에서 액션 목표를 전달할 때 실습으로 사용하였던 /turtle1/rotate_absolute 서비스를 예를 들어 설명하겠다 <br/>
 <br/>
-/turtle1/rotate_absolute 액션에 사용된 RotateAbsolute.action 인터페이스를 알아보기 위해서는 [9]의 원본 파일을 참고해도 되고 <br/>
`ros2 interface show` 명령어를 이용하여 확인할 수 있다 <br/>
-이 명령어를 이용하면 아래의 결괏값과 같이 `turtlesim/action/RotateAbsolute.action`은 float32 형태의 theta, delta, remaining 라는 세개의 데이터가 있음을 알 수 있다 <br/>
여기서 서비스와 마찬가지로 `---` 이라는 구분자를 사용하여 액션 목표(goal)), 액션 결과(result), 액션 피드백(feedback)으로 나누어 사용하게 된다 <br/>
즉 theta는 액션 목표, delta는 액션 결과, remaining는 액션 피드백에 해당된다(참고로 각 데이터는 각도의 SI 단위인 라디안(radian)을 사용한다) <br/>
```
ros2 interface show turtlesim/action/RotateAbsolute.action
```
