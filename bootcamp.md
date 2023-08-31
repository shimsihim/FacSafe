ROS(Robot Operating System)

빌드하기 :  1,2,3, 과정 모두 실행 필요~~~~
ros2 배치파일 실행
 call C:\dev\ros2-windows\setup.bat
터미널 창에서 프로젝트 폴더로 이동 후 빌드(Native를 관리자 권한으로!)
cd C:\Users\SSAFY\Desktop\skeleton\mobility-smarthome-skeleton\ros2_smart_home
colcon build
일부 빌드 시
colcon build --packages-select sub1(선택한 패키지)
빌드 완료 후 노드를 실행, ros실행위해 setup.bat을 call한것 처럼 
워크스페이스 안의 패키지를 사용하기 위해서는 내부의 bat파일 call필요
즉, call C:\dev\ros2-windows\setup.bat
     call C:\Users\SSAFY\Desktop\skeleton\mobility-smarthome-skeleton\ros2_smart_home\install\local_setup.bat
런치파일 있는 곳으로 가서
ros2 launch C:\Users\SSAFY\Desktop\skeleton\mobility-smarthome-skeleton\ros2_smart_home\ssafy_bridge\launch\ssafybridge_launch.py
위의 런치파일은 무조건 틀어놓아야 시뮬레이터와 통신
rqt(시각화 툴)


기존의 것이 아닌 메타 운영체제⇒ os위에 설치하여 os 기능을 이용해 스케줄링, 감시 ,에러처리
⇒ 미들웨어, 소프트웨어 프레임워크라고 함
⇒ 새로운 os가 아님
이번 프로젝트는 ros2를 사용

ros1에서의  tcpros, udpdros가 dds계층으로 바뀜(데이터 디스투리부션 서비스 , 네트워크에 존재하는 통ㅇ신객체 자동 검색, 데이터 연관성에 따라 선택적으로 연결)

노드 ⇒ 최소단위 실행 프로세스
메시지 ⇒ 노드끼리 데이터 주고 받는 것
talker는 메시지를 pub하는 노드, listner는 sub하는 노드
토픽과 메시지 타입을 알아햐 서로 메시지를 주고 받을 수 있음
여기서는 토픽이 chatter , 메시지 타입이 std msgs/String
담긴 내용이 hello World.
리스너가 원하지 않을 때도 토커가 말하면 메시지를 받음 ⇒ 비 동기식 연속성 통신
패키지 ⇒ ros 소프트웨어의 기본 단위 , 빌드단위, 배포단위
여러개의 노드들을 가질 수 있음.
ROS장점

메시지통신을 사용해 하나의 노드에 대해서 공유하지 않고  
복잡한 프로그램을 여러사람이 나눠서 공동 개발에 용이
노드간에 토픽과 메시지 타입을 알려주면 되어서 서로 영향을 주지 않고 쉽게 연결이 가능
또한, 기능별로 만들었기 떄문에 어떤 노드에서 메시지가 나오지 않는지 확인을 통해 오류 확인 편리

실습
바탕화면에 catkin_ws  폴더 만들기
폴더 내에 src폴더 넣기(여기부터가 프로젝트들 넣는 곳)


    src폴더에 들어가서 
    기존에 받아놨던 setup.bat있는 폴더에 해당 파일 호출
     패키지 생성(이름은 my_package)
ros2 pkg create --build-type ament_python --node-name my_node my_pakage
빌드하기


빌드 완료 후 노드를 실행, ros실행위해 setup.bat을 call한것 처럼 
워크스페이스 안의 패키지를 사용하기 위해서는 내부의 bat파일 call필요
즉, call C:\dev\ros2-windows\setup.bat
     call C:\Users\SSAFY\Desktop\catkin_ws\install\local_setup.bat
2개는 명령창 킬 떄 마다 반복


노드 실행 (ros2 run my_package my_node)

나의 노드 직접 만들기(https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
my_package패키지의 my_package폴더 안에 publisher_member_function.py만들고 
위의 링크의 publisher_member_function.py 복붙

8.이제 스크립트가 생성되었지만 노드가 생성된 것은 아님 ⇒ 패키지 내에 setup.py파일에 우리가 만든 스크립트 이름을 정의해야 함

이후 새롭게 빌드해야 함


위의 것은 SubScriber 를 만드는 법

명령프롬프트 킬 떄 마다 ros2명령어 치기 는 불가능
런치파일 실행도 런치파일이 있는 곳으로 가서 
ros2 launch test_launch.py

