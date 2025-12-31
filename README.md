# ros2-controller-sample-cpp
> ⚠️ **주의: 이 노드는 반드시 ROS2 Humble에서만 테스트되었습니다.**  
> ⚠️ **주의: 이 프로젝트는 리눅스 환경에서만 테스트 되었습니다.**  
> ⚠️ **주의: 이 README.md에서 ros2 설치 방법은 나오지 않습니다.**

## 1. ros2 초기 설정

## 2. ros2 빌드

### 1. 터미널 열기

### 2. 빌드 명령어 입력
```bash
cd ros2-controller-sample-cpp # 디렉토리 이동
colcon build # 패키지 빌드
```

### 3. 노드 실행 및 테스트

```bash
# 첫번째 터미널 ( NodeA 실행 )
cd ros2-controller-sample-cpp
ros2 run test_nodes node_a
```

```bash
# 두번째 터미널 ( NodeB 실행 )
cd ros2-controller-sample-cpp
ros2 run test_nodes node_b
```

```bash
# 세번째 터미널 ( Ros2 WebSocket 서버 실행 )
cd ros2-controller-sample-cpp
sudo apt update
sudo apt install ros-humble-rosbridge-suite # 패키지 설치 
ros2 launch rosbridge_server  rosbridge_websocket_launch.xml # 서버 실행
```

## 3. 웹 브라우저에서 확인

### 1. 터미널 실행

### 2. 서버 실행
```bash
cd ros2-controller-sample-cpp
python3 -m http.server 8000
```

### 3. 웹 브라우저 접속
-  로컬 노트북 or 가상환경 브라우저에서 접속
```bash
http://192.168.189.132:8000/web_client.html
```