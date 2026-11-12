# collector-amr-ros2

CAMTIC ST-mini(Scout Mini)의 ROS 2 토픽을 읽어, 다른 collector 와 같은 ZeroMQ 로 엣지 미들웨어에 올린다.

```
camtic_dev (ROS 2 Humble)
  /scout_status  /wheel_odom  /amcl_pose  /scan
        │
        ▼
collector-amr-ros2  ── PUB tcp://0.0.0.0:5570
        │  collector.robot.amr-scout-1.{data|health}
        ▼
middleware_edge_forwarder  ── OPC UA / MQTT
```

드라이버는 띄우지 않는다. `camtic_bringup` 또는 `slam_bringup` 이 이미 토픽을 내고 있어야 값이 나온다. 라이다 배열과 지도 이미지는 보내지 않는다. 미들웨어가 배열 샘플을 받지 않는다.

## 실리는 값

| addr | 내용 |
| --- | --- |
| `battery_voltage`, `error_code`, `linear_velocity`, `angular_velocity` | `/scout_status` 의 스칼라. 필드 이름은 메시지 슬롯을 그대로 쓴다 |
| `actuator_states_0_temperature` 처럼 인덱스 붙은 이름 | 길이 8 이하인 상태 배열만 |
| `odom_x` `odom_y` `odom_yaw` `odom_vx` `odom_wz` | `/wheel_odom`. yaw 는 rad |
| `amcl_x` `amcl_y` `amcl_yaw` | `/amcl_pose`. 자율주행일 때만 |
| `scan_ranges` `scan_intensities` | `/scan` 최신 프레임. `f64[]`. 무한대는 `range_max`/`range_min`으로 바꾼다 |
| `scan_angle_min` `scan_angle_max` `scan_range_min` `scan_range_max` | `/scan` 각도·거리 한계 |
| `wheel_odom_ok` `scout_status_ok` `amcl_ok` `scan_ok` | 최근 `STALE_SEC` 안에 메시지가 있었는지 |

`/wheel_odom` 과 `/scout_status` 가 둘 다 끊기면 DATA 를 멈추고 HEALTH `device_ok=false`, `reason=E-1003` 을 보낸다. SLAM 중에는 `amcl_ok=false` 인 채로 나머지를 보낸다.

토픽은 `collector.robot.{DEVICE_KEY}.data` / `.health` 이다. 샘플 필드는 `addr`, `name`, `type`, `value`, `error` 이고 `sockets` 는 `[]` 이다.

## 로봇에 올리기

저장소는 `/home/camtic/collector-amr-ros2` 에 둔다. 이 경로는 로봇 PC와 `camtic_dev` 가 같이 본다. `camtic_ws` 안에는 넣지 않는다.

```bash
# 로봇 PC (camtic@). 컨테이너 안이면 exit 로 한 번 나온다.
cd /home/camtic
git clone <이 저장소 주소> collector-amr-ros2
cd collector-amr-ros2
cp .env.example .env
```

개발 모드로 바퀴와 센서가 떠 있는지 먼저 확인한다. 운영 모드(`camtic-robot-nav`, `camtic-robot-zenoh`)와 bringup 을 동시에 켜지 않는다.

```bash
# camtic_dev 안
ros2 topic hz /wheel_odom          # 약 50Hz
ros2 topic echo --once /scout_status
```

그 다음 같은 컨테이너에서 collector 를 띄운다. `run.sh` 가 ROS 환경, 떠 있는 드라이버의 `ROS_DOMAIN_ID` / RMW, pip 패키지를 맞춘다.

```bash
# camtic_dev 안
cd /home/camtic/collector-amr-ros2
./run.sh
```

정상이면 1초마다 이런 로그가 나온다.

```text
subscribe /wheel_odom type=nav_msgs/msg/Odometry publishers=1
subscribe /scout_status type=scout_msgs/msg/ScoutStatus publishers=1
sample addrs: odom_x, odom_y, ..., battery_voltage, error_code, wheel_odom_ok, scan_ok
[DATA] {"msg_type":"data","collector_address":"amr-scout-1",...}
```

다른 터미널에서 프레임만 보려면:

```bash
# camtic_dev 안
python3 -m core.zeromq_client --device-key amr-scout-1
```

`토픽 대기: /wheel_odom=no_publisher` 가 반복되면 bringup 이 이 컨테이너 그래프에 없는 것이다. `ros2 topic list` 에 `/wheel_odom` 이 보이는 곳에서 `./run.sh` 를 실행한다. 대화형 셸의 `echo $ROS_DOMAIN_ID` 와 collector 로그의 `ROS_DOMAIN_ID=` 가 다르면 `.env` 에 대화형 셸 값을 적는다.

## 서비스 등록

로봇 PC에서 등록한다. 컨테이너 안에서는 등록하지 않는다. `host_start.sh` 가 `camtic_dev` 안에서 `run.sh` 를 실행한다. 컨테이너 네트워크가 host 가 아니면 호스트의 5570 을 컨테이너 PUB 으로 넘긴다.

수동으로 띄운 `./run.sh` 는 먼저 Ctrl+C 로 끈다. 같은 포트를 서비스가 다시 열지 못한다.

```bash
# 로봇 PC
cd /home/camtic/collector-amr-ros2
sudo bash service_add.sh
journalctl -u collector-amr-ros2 -f
```

로봇 PC에서 포트가 보여야 엣지가 붙는다.

```bash
# 로봇 PC
ss -ltnp | grep 5570
```

제거:

```bash
sudo systemctl disable --now collector-amr-ros2
sudo rm /etc/systemd/system/collector-amr-ros2.service
sudo systemctl daemon-reload
```

`run.sh` 가 컨테이너 파이썬에 설치한 pyzmq, pydantic 은 컨테이너를 지우고 다시 만들면 사라진다. 다음 기동 때 `run.sh` 가 다시 설치한다.

## 엣지 미들웨어

`middleware_edge_forwarder` 의 `ZMQ_COLLECTOR_KINDS` 에 `robot` 이 이미 있으면 종류는 그대로 둔다. 구독 주소만 로봇 IP 를 추가하고 포워더를 다시 시작한다. 유선 주소는 공식 매뉴얼 기준 `192.168.1.211` 이다. SSH 에 쓰는 주소와 같아야 한다.

```text
ZMQ_SUB_ENDPOINTS=tcp://127.0.0.1:5557,tcp://127.0.0.1:5558,tcp://127.0.0.1:5559,tcp://127.0.0.1:5560,tcp://127.0.0.1:5561,tcp://192.168.1.211:5570
```

포워더 로그에 `DATA collector.robot.amr-scout-1` 이 보이면 연결됐다. auto 노드가 켜져 있으면 OPC UA 경로는 `Collectors/robot/amr-scout-1/<addr>` 이다.

## 로컬 스모크

ROS 없이 샘플 변환과 ZeroMQ 프레임을 확인한다.

```bash
python script/smoke_samples.py
```

마지막 줄이 `== PASS 6/6 ==` 이면 성공이다.
