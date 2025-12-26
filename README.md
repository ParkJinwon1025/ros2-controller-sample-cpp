# ros2-controller-sample-cpp

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

### 1. web_client.html 실행

### 2. 웹페이지에서 버튼 클릭 후 `node_a`, `node_b` 터미널 로그 확인