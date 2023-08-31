# PJT1 명세서 학습

## 학습 내용
- 명세서 sub1 학습 및 스켈레톤 코드 학습
- YOLO + OpenCV 이론 학습 및 코드 학습

### 학습 내용

- ROS2 메시지 통신에 대한 이해와 실습
- 영상 데이터 수신하여 OpenCV 라이브러리 활용
- 로봇 상태 정보 수신 및 제어 데이터 전송
- 디바이스 상태 정보 수신 및 제어 데이터 전송
- 환경 상태 정보 수신 및 출력
- custom Object Hand control 제어 데이터 전송
- IMU 센서 이해 및 활용
- 주행기록계 만들기
- 주행기록 기반 경로 생성 및 저장
- 경로추종 Follow the Carrot

# 이론

rvis2 → 라이다, tf, image 등의 메시지를 3D로 시각화

rqt → 토픽으로 나오는 메시지를 모니터할 수 있으며, 노드와 메시지와의 관계를 그래프로 그려줌.

## 스켈레톤 코드 학습

### 파일 구조

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled.png)

아래의 노드들은 sub1 폴더 하위에 있음

- publisher → String 메시지를 publish 하는 노드
- subcriber → String 메시지를 subscribe 하는 노드
- controller → IoT 기기 메시지를 publish, subscribe 하는 노드
- perception → 영상 메시지를 받아서 영상처리를 하는 노드
- handcontrol → Hand Control 메시지를 publish 하는 노드
- odom  → 로봇의 센서를 이용해 로봇의 Odometry(상대위치)를 계산하는 노드
- make_path → Odometry 를 이용해 경로를 텍스트 파일로 기록하는 노드
- path_pub → 저장한 텍스트 파일을 읽어서 ROS 메시지로 publish 하는 노드
- path_tracking → 로봇의 상태, 위치, 경로 등을 받아서 경로를 추종하는 노드

### 시스템 아키텍쳐

<aside>
✔️ ssafybridge 는 시뮬레이터와 통신하게 해주는 런치파일

</aside>

> ROS 메세지 통신 노드 실행
> 

publisher → (/test) → subscriber

> IoT 기기의 상태, 제어 메시지 송수신
> 

ssafybridge_launch → (/turtlebot_status) → controller

ssafybridge_launch ← (/app_control) ← controller

> 카메라 데이터 수신 및 영상처리
> 

ssafybridge_launch → (/image_jpeg/compressed) → perception

> Hand Control
> 

ssafybridge_launch → (/turtlebot_status) → handcontrol

ssafybridge_launch ← (/hand_control) ← handcontrol

> 주행기록계, 경로만들기, 경로읽어오기 및 경로추종은 명세서 참조
> 

### 기본 코드 설명

> publisher
> 

> subscriber
> 

> controller
> 
- 시뮬레이터로부터 데이터를 수신해서 확인(출력)하고, 메시지를 송신해서 제어가 되는지 확인해보는 통신 테스트를 위한 노드
- 수신할 데이터 ⇒ 터틀봇 상태(/turtlebot_status), 환경정보(/envir_status), 가전정보(/app_status)
- 송신할 데이터 ⇒ 터틀봇 제어(/ctrl_cmd), 가전제어(/app_control)

> perception
> 
- 시뮬레이터에 달린 카메라로 찍힌 이미지를 메시지 통신을 사용하여 compressed image 형태로 받아서 보여주는 노드
- 수신할 이미지를 opencv 의 imshow 로 윈도우 창에 띄우는 역할

## 코드실습

### (1) ROS 메시지 통신 노드 실행

`call C:\dev\ros2-windows\setup.bat`

`call C:\Users\SSAFY\Desktop\mobility-smarthome-skeleton\install\local_setup.bat`

`ros2 run sub1 publisher`

`ros2 run sub1 subscriber`

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled%201.png)

### (2) controller

**⇒ ssafy_bridge 파일 launch**

ros2 run 패키지명 스크립트 이름(파일명)

`ros2 launch C:\Users\SSAFY\Desktop\mobility-smarthome-skeleton\ros2_smart_home\ssafy_bridge\launch\ssafybridge_launch.py`

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled%202.png)

**⇒ run 컨트롤러**

`ros2 run sub1 controller`

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled%203.png)

**⇒ rqt 실행**

`rqt`

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled%204.png)

---

call c:\dev\ros2-windows\setup.bat

call C:\Users\SSAFY\Desktop\mobility-smarthome-skeleton\install\local_setup.bat

`ros2 run sub1 controller`

---

`ros2 launch c:\Users\SSAFY\Desktop\mobility-smarthome-skeleton\ros2_smart_home\ssafy_bridge\launch\ssafybridge_launch.py`

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled%205.png)

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled%206.png)

### 카메라 인식

`ros2 run sub1 perception`

```python
#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import CompressedImage

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow

class IMGParser(Node):
    def __init__(self):
        super().__init__(node_name="image_convertor")

        # 로직 1. image subscriber 생성
        ## 아래와 같이 subscriber가
        ## 미리 정의된 토픽 이름인 '/image_jpeg/compressed' 에서
        ## CompressedImage 메시지를 받도록 설정된다.

        self.subscription = self.create_subscription(
            CompressedImage, "/image_jpeg/compressed", self.img_callback, 10
        )

    def img_callback(self, msg):
        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        """
        로직 3. 이미지 색 채널을 gray scale로 컨버팅
        cv2. 내의 이미지 색 채널 컨터버로 bgr 색상을 gary scale로 바꾸십시오.
        """
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        """
        로직 4. 이미지 resizing
        cv2를 사용해서 이미지를 원하는 크기로 바꿔보십시오.
        """
        img_resize = cv2.resize(
            img_gray, dsize=(100, 100), interpolation=cv2.INTER_LINEAR
        )

        # 로직 5. 이미지 출력 (cv2.imshow)

        cv2.imshow("img_bgr", img_bgr)
        cv2.imshow("img_gray", img_gray)
        cv2.imshow("resize and gray", img_resize)

        cv2.waitKey(1)

def main(args=None):
    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의
    image_parser = IMGParser()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin(image_parser)

if __name__ == "__main__":
    main()
```

![Untitled](PJT1%20%E1%84%86%E1%85%A7%E1%86%BC%E1%84%89%E1%85%A6%E1%84%89%E1%85%A5%20%E1%84%92%E1%85%A1%E1%86%A8%E1%84%89%E1%85%B3%E1%86%B8%20c7cca15d611d46e5a2564755870cc7a4/Untitled%207.png)



# YOLO

### **Object Detection, 객체 인식**

- 이미지 또는 비디오에서 개체를 식별하고 찾는 것과 관련된 컴퓨터 비전 작업
- Object Detection 기술은 두 가지 질문을 위해 존재
    - **이것은 무엇인가?** 특정 이미지에서 대상을 식별하기 위함
    - **어디에 위치해있는가?** 이미지 내에서 개체의 정확한 위치를 설정하기 위함

### YOLO 의 정의와 개념

= **You Only Look Once**

- 최첨단 실시간 Object Detection 시스템
- 물체 감지와 객체 인식에 대한 딥러닝 기반 접근 방식
- YOLO는 입력된 이미지를 일정 분할로 그리드한 다음, 신경망을 통과하여 [바운딩 박스](https://www.thedatahunt.com/tech-review/how-to-improve-data-quality-in-ocr)와 클래스 예측을 생성하여 최종 감지 출력을 결정
    - Bbox를 계산하기 위해 YOLO는 IoU(Intersect over Union) 및 NMS(Non-maximum suppression)의 주요 후처리 단계를 구현
    - NMS는 이미지에 얼굴이 여러 개 있다고 판단하는 것이 아니라, 동일한 객체에 대한 상자 중 가장 높은 확률을 가진 상자를 선택
    - 즉, 아래 사진과 같이 모든 bounding box 에 object 가 존재하지만,  가장 높은  objectiveness score 을 가지는 녹색 box 만을 선택한다.
    
    ![Untitled](YOLO%20ad9e11a23ba24c469d9a690296942c29/Untitled.png)
    

NMS 관련 참고사이트:

[https://www.analyticsvidhya.com/blog/2020/08/selecting-the-right-bounding-box-using-non-max-suppression-with-implementation/](https://www.analyticsvidhya.com/blog/2020/08/selecting-the-right-bounding-box-using-non-max-suppression-with-implementation/)

[Selecting the Right Bounding Box Using Non-Max Suppression (with implementation)](https://www.analyticsvidhya.com/blog/2020/08/selecting-the-right-bounding-box-using-non-max-suppression-with-implementation/)

[https://velog.io/@qtly_u/d5isa9ts](https://velog.io/@qtly_u/d5isa9ts)

[[Object Detection] NMS(Non Maximum Suppression), Soft NMS](https://velog.io/@qtly_u/d5isa9ts)

### **YOLO 에 주목해야 하는 이유 ⇒ 활용도**

- YOLO는 빠른 속도와 상대적으로 높은 정확도를 자랑 ⇒ `실시간성`
    - **속도**: 물체를 실시간으로 예측하여 감지 속도 향상
    - **높은 정확도**: 최소한의 배경 오류로 정확한 결과를 제공
    - **학습 기능**: YOLO는 객체의 표현을 학습하고 이를 객체 감지에 적용할 수 있는 뛰어난 학습 기능

### YOLO 버전

- 사용할 모델을 결정하는 것은 애플리케이션의 요구 사항에 따라 달라지지만, 실시간 Object Detection 작업이 필요할 경우 `YOLOv8`을 선택하는 경향

YOLO 개념 관련 참고 사이트

[https://www.thedatahunt.com/trend-insight/guide-for-yolo-object-detection](https://www.thedatahunt.com/trend-insight/guide-for-yolo-object-detection)

# 실습

### 0) OpenCV와 numpy 모듈을 import

```python
import cv2
import numpy as np
```

### 1) YOLO 로드

```python
# Load Yolo
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))
```

**YOLO 알고리즘을 실행하기 위해서 필요한 세 개의 파일**

- Weight file : 훈련된 model
- Cfg file : 구성파일. 알고리즘에 관한 모든 설정.
- Name files : 알고리즘이 감지할 수 있는 객체의 이름을 포함

### 2) 물체를 감지할 이미지 로드

```python
# Loading image
img = cv2.imread("a101.jpg")
img = cv2.resize(img, None, fx=0.4, fy=0.4)
height, width, channels = img.shape
```

### 3) 이미지를 Blob 으로 변환

YOLO가 허용하는 세가지 크기

- 320 × 320 : 작고 정확도는 떨어지지 만 속도 빠름
- 609 × 609 : 정확도는 더 높지만 속도 느림
- 416 × 416 : 중간

### 4) 물체 감지

```python
# Detecting objects
blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

net.setInput(blob)
outs = net.forward(output_layers)
```

outs 는 감지결과. 탐지된 개체에 대한 모든 정보와 위치 제공

### 5)  결과 정보를 화면에 표시

```python
# Showing informations on the screen
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)

            # Rectangle coordinates
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)

            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
print(indexes)
```

- 신뢰도가 0.5 이상이라면 물체가 정확히 감지되었다고 간주
- 임계값은 0에서 1사이의 값을 가지며, 1에 가까울수록 탐지 정확도가 높고 , 0에 가까울수록 정확도는 낮아지지만 탐지되는 물체의 수는 많아짐

### 6) 사용자 화면에 보여주기

```python
font = cv2.FONT_HERSHEY_PLAIN
for i in range(len(boxes)):
    if i in indexes:
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        color = colors[class_ids[i]]
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.putText(img, label, (x, y + 30), font, 3, color, 3)

cv2.imshow("Image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

## 실습결과

참고 ) 실습은 YOLOv3 로 진행함

### test1 → laptop, cup, keyboard

![Untitled](YOLO%20ad9e11a23ba24c469d9a690296942c29/Untitled%201.png)

### test2 → person

![Untitled](YOLO%20ad9e11a23ba24c469d9a690296942c29/Untitled%202.png)

![Untitled](YOLO%20ad9e11a23ba24c469d9a690296942c29/Untitled%203.png)

> 참고사이트
> 

1.[https://pysource.com/2019/06/27/yolo-object-detection-using-opencv-with-python/](https://pysource.com/2019/06/27/yolo-object-detection-using-opencv-with-python/)

[YOLO object detection using Opencv with Python - Pysource](https://pysource.com/2019/06/27/yolo-object-detection-using-opencv-with-python/)

1. [https://bong-sik.tistory.com/16](https://bong-sik.tistory.com/16)

[Python으로 OpenCV를 사용하여  YOLO Object detection](https://bong-sik.tistory.com/16)