# Turtlesim Action
Reference: https://cafe.naver.com/openrt/24142 <br/>

## Run Node
### 1.Run Turtlesim Node
```
roscore
```

### 2.ssh

```
# E4:5F:01:D2:72:C8

ssh ubuntu@192.168.0.35
```

### 3.bashrc 

```
nano ~/.bashrc

# MASTER = PC IP
# HOSTNAME = TURTLE IP

MASTER = 192.168.0.61
HOSTNAME = 192.168.0.35

source ~/.bashrc
```

### 4.bringup

```
export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
### 5.bashrc

```
nano ~/.bashrc 


```
