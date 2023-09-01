# TurtleBot3 ROS2 Foxy 설치

# 1.PC Setup

VM ware 설치 - [https://www.vmware.com/kr/products/workstation-player/workstation-player-evaluation.html](https://www.vmware.com/kr/products/workstation-player/workstation-player-evaluation.html)

Ubuntu 20.04버전 설치 - [https://releases.ubuntu.com/20.04/](https://releases.ubuntu.com/20.04/)

vm ware 실행후 - player→File→New Virtual Machine 누르고 우분투 파일 넣고 실행

**ros2 설치**

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
$ sudo chmod 755 ./install_ros2_foxy.sh
$ bash ./install_ros2_foxy.sh
```

****[Install Dependent ROS 2 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-dependent-ros-2-packages)****

**ROS2 종속 패키지 설치**

- Gazebo 11 설치 ( 시뮬레이션이라 필요없을수도)

`$ sudo apt-get install ros-foxy-gazebo-*`

- Cartographer 설치

```
$ sudo apt install ros-foxy-cartographer
$ sudo apt install ros-foxy-cartographer-ros
```

- Navigation2 설치

```
$ sudo apt install ros-foxy-navigation2
$ sudo apt install ros-foxy-nav2-bringup
```

T**urtleBot3 패키지 설치**

- Install TurtleBot3 via Debian Packages.

```
$ source ~/.bashrc
$ sudo apt install ros-foxy-dynamixel-sdk
$ sudo apt install ros-foxy-turtlebot3-msgs
$ sudo apt install ros-foxy-turtlebot3
```

**환경 구성**

- Set the ROS environment for PC.

```
$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc
```

# 2. SBC Setup

- Raspberry Pi 3B+ ROS2 Foxy image 다운

-[https://www.robotis.com/service/download.php?no=2058](https://www.robotis.com/service/download.php?no=2058)

(Raspberry Pi 4라면 [https://www.robotis.com/service/download.php?no=2064](https://www.robotis.com/service/download.php?no=2064))

- Raspberry Pi Imager 설치 -[https://www.raspberrypi.com/software/](https://www.raspberrypi.com/software/)
- sd card Formatter로 포멧한번하기
- Micro SD 이미지파일 굽기

 →운영체제 누르기 → Use custom → foxy다운받은 image선택 →select Micro SD → Write

- wifi 네트워크 설정

터미널 창에서 입력

```
$ cd /media/$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml
```

![Untitled](TurtleBot3%20ROS2%20Foxy%20%E1%84%89%E1%85%A5%E1%86%AF%E1%84%8E%E1%85%B5%20ae72420d5a574c32bd94d5b9695e5e7c/Untitled.png)

저장 후 라즈베리파이에 MicroSD카드 삽입후 키보드와 모니터를 연결한다. 그 후 ubuntu에 id와 password입력 후 로그인

- ROS2 네트워크 구성
    
    -ROS2 DDS 통신에는 동일한 네트워크 환경에서 통신하기 위해서는Remote PC와 TurtleBot3의 ROS_DOMAIN_ID가 일치해야한다. TurtleBot3의 기본 ROS도메인 ID는 ***.bashrc*** 파일 에 설정되어 있습다 . 동일한 네트워크에 동일한 ID가 존재할 경우 충돌을 피하기 위해 ID를 수정해야함.
    
    ![Untitled](TurtleBot3%20ROS2%20Foxy%20%E1%84%89%E1%85%A5%E1%86%AF%E1%84%8E%E1%85%B5%20ae72420d5a574c32bd94d5b9695e5e7c/Untitled%201.png)
    
- LDS-02구성

TurtleBot3 LDS는 2022년 모델부터 LDS-02로 업데이트되었습니다.

1. LDS-02 드라이버 설치 및 TurtleBot3 패키지 업데이트

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/turtlebot3_ws/src
$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws && colcon build --symlink-install
```

# sudo apt update시 Error가 발생시

→ 이는 저장소의 릴리스 파일이 현재 시간으로부터 지난 시간까지 유효하지 않음

1. **시간 설정 확인 및 조정**:

$ date
$ sudo timedatectl set-time "YYYY-MM-DD HH:MM:SS"

→ Failed to set time: Automatic time synchronization is enabled 에러 발생

→ 시스템이 자동으로 시간 동기화가 활성화되어 있어 수동으로 시간을 설정할 수 없다는 것을 나타냄. 이 경우, 시스템의 자동 시간 동기화를 비활성화한 후에 시간을 설정해야 함.

1. **자동 시간 동기화 비활성화:**    

$ sudo timedatectl set-ntp off

1. **수동으로 시간 설정:**

$ sudo date --set "YYYY-MM-DD HH:MM:SS”

1. **저장소 업데이트 및 패키지 업데이트**:

$ sudo apt update
$ sudo apt upgrade

이후 다시 LDS-02 드라이버 설치하면된다.

- LDS_MODEL을 bashrc 파일로 내보냅니다. LDS 모델에 따라 LDS-01 or 02를 사용하면 된다.

```
$ echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc
```

# 3. Open CR 설정

- 마이크로usb 케이블을 사용하여 OpenCR을 Raspberry Pi에 연결
- Open CR 펌웨어를 업로드하기위해 Raspberry Pi에 필수 패키지 설치

```
$ sudo dpkg --add-architecture armhf
$ sudo apt update
$ sudo apt install libc6:armhf
```

- 플랫폼에 따라 **OPENCR_MODEL** 이름 에 burger 또는 waffle 중 하나를 사용.(우리는 버거)

```
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2
```

- 펌웨어와 로더를 다운로드한 다음 파일을 추출한다

```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
$ tar -xvf ./opencr_update.tar.bz2
```

- OpenCR에 펌웨어를 업로드한다

-성공적인 업로드 화면

![Untitled](TurtleBot3%20ROS2%20Foxy%20%E1%84%89%E1%85%A5%E1%86%AF%E1%84%8E%E1%85%B5%20ae72420d5a574c32bd94d5b9695e5e7c/Untitled%202.png)

# 4. Bring up

- 이제 PC에서 Raspberry pi와 연결을 시도하자

`$ ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}`

우리는 $ ssh [ubuntu@192.168.137.12](mailto:ubuntu@192.168.137.12)

- TurtleBot3 애플리케이션을 시작하기위해 기본 패키지를 불러오기

```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_bringup robot.launch.py
```

![Untitled](TurtleBot3%20ROS2%20Foxy%20%E1%84%89%E1%85%A5%E1%86%AF%E1%84%8E%E1%85%B5%20ae72420d5a574c32bd94d5b9695e5e7c/Untitled%203.png)

대충 이런 창 뜨면 성공한거임

`$ ros2 topic list` → Topic List 확인

`$ ros2 service list` → Service List 확인