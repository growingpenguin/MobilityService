# Turtlesim Service
Reference: https://cafe.naver.com/openrt/24128 <br/>

## Run Node
### 1.Run Turtlesim Node
```
ros2 run turtlesim turtlesim_node
```

### 2.Service Commands

**list** <br/>
현재 실행 중인 노드들의 서비스 목록 <br/>
```
ros2 service list
```
다양한 서비스들이 포함되어 있는데 parameters 가 붙어있는 서비스는 파라미터와 관련된 내용으로 모든 노드의 기본 기능으로 포함되어 있다 <br/>

**type** <br/>
```
ros2 service type /clear
# std_srvs/srv/Empty

ros2 service type /kill
# turtlesim/srv/Kill

ros2 service type /spawn
# turtlesim/srv/Spawn
```

**list -t** <br/>
```
ros2 service list -t
```

**find** <br/>
```
ros2 service find std_srvs/srv/Empty

ros2 service find turtlesim/srv/Kill
```

**call** <br/>
```
 ros2 service call <service_name> <service_type> "<arguments>"
```
```
ros2 run turtlesim turtle_teleop_key

ros2 service call /clear std_srvs/srv/Empty
```
/clear 서비스를 요청해보자 <br/>
이동 궤적이 서비스 요청 후에는 모두 지워짐을 확인할 수 있다 <br/>
아래 명령어에서 "<arguments>" 가 생략되었는데 이는 std_srvs/srv/Empty 이라는 서비스 형태가 아무런 내용이 없는 형태로 사용할 수 있기 때문이다 <br/>
<br/>
```
ros2 service call /kill turtlesim/srv/Kill "name: 'turtle1'"
```
/kill 서비스를 요청해보자 <br/>
/kill 서비스는 죽이고자하는 거북이 이름을 서비스 요청의 내용으로 입력하면되는데 아래과 같이 turtle1이라고 이름을 지정하면 그림 5와 같이 거북이가 사라졌음을 확인할 수 있다 <br/>
<br/>
```
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 255, b: 255, width: 10}"
```
/set_pen 서비스를 요청하자 <br/>
이 서비스는 지정한 거북이의 궤적 색과 크기를 변경하는 것으로 아래와 같이 r, g, b 값을 조합하여 색을 지정하고, width로 궤적의 크기를 지정할 수 있다 <br/>
이를 여러번 값을 변경해보며 테스트하면 그림 7과 같다 <br/>

```
ros2 service call /kill turtlesim/srv/Kill "name: 'turtle1'"

ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 9, theta: 1.57, name: 'leonardo'}"

ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 7, theta: 1.57, name: 'raffaello'}"

ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5, theta: 1.57, name: 'michelangelo'}"

ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 3, theta: 1.57, name: 'donatello'}"

ros2 topic list
```
/spawn 서비스 요청 <br/>
지정한 위치 및 자세에 지정한 이름으로 거북이를 추가시키게 된다 <br/>
이름은 옵션으로 지정하지 않으면 turtle2처럼 자동으로 지정되며 동일한 이름을 사용할 수는 없다 <br/>

**interface show srv** <br/>
```
ros2 interface show turtlesim/srv/Spawn.srv
```
/spawn 서비스에 사용된 Spawn.srv 인터페이스를 알아보자 <br/>
`ros2 interface show` 명령어를 이용하여 확인할 수 있다 <br/> 
이 명령어를 이용하면 아래의 결괏값과 같이 `turtlesim/srv/Spawn.srv`은 float32 형태의 x, y, theta 그리고 string 형태로 name 이라고 두개의 데이터가 있음을 알 수 있다. 여기서 특이한 것은 `---` 이다. 이는 구분자라 부른다 <br/>
서비스 인터페이스는 메시지 인터페이스와는 달리 서비스 요청 및 응답(Request/Response) 형태로 구분되는데 요청(Request)과 응답(Response)을 나누어 사용하기 위해서 `---`를 사용하게 된다. 즉 x, y, theta, name 은 서비스 요청에 해당되고 서비스 클라이언트 단에서 서비스 서버단에 전송하는 값이 된다 <br/>
그리고 서비스 서버단은 지정된 서비스를 수행하고 name 데이터를 서비스 클라이언트단에 전송 <br/>
<br/>
실제로 우리는 /spawn 서비스를 사용할 때 신규 거북이를 생성하기 위하여 x, y, theta 로 위치와 자세를 지정하고, name에 신규 거북이의 이름을 지정하였다. 그 뒤 거북이가 추가되면서 추가된 거북이의 이름을 반환 받았었다
<br/>
