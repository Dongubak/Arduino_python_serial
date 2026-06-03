# Gunsan 자율주행 차선 인식 시스템

> **상태:** 개발 중 (카메라 단일 테스트 단계)  
> **목적:** 유아용 자동차 기반 트랙 자율주행 — 카메라 차선 인식 → 조향 제어  
> **ROS 버전:** ROS 2 Humble  

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [하드웨어 구성](#2-하드웨어-구성)
3. [소프트웨어 구조](#3-소프트웨어-구조)
4. [카메라 내부 파라미터 (캘리브레이션)](#4-카메라-내부-파라미터-캘리브레이션)
5. [차선 인식 처리 파이프라인](#5-차선-인식-처리-파이프라인)
6. [카메라 방향 기준선 정렬](#6-카메라-방향-기준선-정렬)
7. [ROI 설정 방법](#7-roi-설정-방법)
8. [실행 방법](#8-실행-방법)
9. [파라미터 레퍼런스](#9-파라미터-레퍼런스)
10. [향후 개발 계획](#10-향후-개발-계획)

---

## 1. 시스템 개요

```
[카메라] ──image_raw──► [lane_detector_node] ──/lane_error──► [조향 제어 노드] (미구현)
                                │
                         /lane_debug (디버그 이미지)
```

- 카메라로 촬영한 트랙 이미지에서 차선을 검출하여 **정규화된 조향 오차**(`/lane_error`, -1.0 ~ +1.0)를 계산한다.
- 오차 0.0 = 차선 중앙, 음수 = 좌측 이탈, 양수 = 우측 이탈.
- 현재 단계: **카메라 단일 테스트** (브라켓 미제작으로 전면 카메라 1대만 사용)

---

## 2. 하드웨어 구성

| 항목 | 사양 | 비고 |
|------|------|------|
| 카메라 | Logitech C920e × 2 | 현재 전면 1대만 테스트 |
| 라이다 | RPLIDAR A1M8 | 카메라 작업 후 연결 예정 |
| 구동부 | 후륜 DC모터 × 2 (직결) | 드라이버 미장착 (개발 예정) |
| 조향 | DC모터 + 가변저항 | PWM 제어 예정 |
| MCU | Arduino (시리얼 통신) | 기존 `robot_control.py` 참고 |
| 플랫폼 | 유아용 자동차 | 본넷에 카메라 장착 |

### 카메라 디바이스 확인

카메라 연결 후 아래 명령으로 실제 `/dev/videoN` 번호를 확인한다.

```bash
v4l2-ctl --list-devices
```

출력 예시:
```
C920e (usb-0000:00:14.0-4):
    /dev/video4
    /dev/video5
```
이 경우 `video_device`는 `/dev/video4`(첫 번째 항목)를 사용한다.

---

## 3. 소프트웨어 구조

```
Gunsan_ws/
└── src/
    └── gunsan_drive/
        ├── config/
        │   ├── camera5_info.yaml       ← camera_info_manager 캘리브레이션 (camera5)
        │   ├── camera6_info.yaml       ← camera_info_manager 캘리브레이션 (camera6)
        │   ├── camera5_params.yaml     ← usb_cam 노드 파라미터
        │   └── camera6_params.yaml
        ├── launch/
        │   ├── cameras.launch.py       ← 카메라 2대 구동 (독립 실행 가능)
        │   └── lane_detection.launch.py ← 카메라 + 차선 검출 노드
        └── gunsan_drive/
            └── lane_detector_node.py   ← 차선 검출 핵심 노드
```

### ROS 2 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/camera5/image_raw` | `sensor_msgs/Image` | 입력 | usb_cam 원본 이미지 |
| `/camera5/camera_info` | `sensor_msgs/CameraInfo` | 입력 | 캘리브레이션 정보 |
| `/lane_error` | `std_msgs/Float32` | 출력 | 정규화 조향 오차 (-1~+1) |
| `/lane_debug` | `sensor_msgs/Image` | 출력 | 좌: 원본+ROI / 우: 버드아이+라인 |

---

## 4. 카메라 내부 파라미터 (캘리브레이션)

캘리브레이션은 **렌즈 왜곡 보정**과 **3D → 2D 투영** 계산에 필요하다.  
원본 파일 위치: `~/logitech camera intrinsic calibratin value/`

### Camera 5 파라미터

| 항목 | 값 |
|------|----|
| 해상도 | 1280 × 720 |
| fx (수평 초점거리) | 1001.364 px |
| fy (수직 초점거리) | 1000.245 px |
| cx (주점 x) | 661.512 px |
| cy (주점 y) | 360.654 px |
| k1, k2 (방사 왜곡) | 0.0589, -0.1603 |
| p1, p2 (접선 왜곡) | 0.0016, -0.0038 |

### 카메라 행렬(K) 의미

```
K = [ fx   0  cx ]
    [  0  fy  cy ]
    [  0   0   1 ]
```

- **fx, fy**: 렌즈의 초점거리를 픽셀 단위로 표현. 값이 클수록 줌인 효과.
- **cx, cy**: 이미지 중심점(주점). 이상적으로는 `(width/2, height/2)`이지만 실제 렌즈 편심으로 차이가 있음.

### 왜곡 계수(D) 의미

```
D = [k1, k2, p1, p2, k3]
```

- **k1, k2, k3**: 방사형(Radial) 왜곡 — 이미지 가장자리가 오목/볼록하게 휘는 현상
- **p1, p2**: 접선(Tangential) 왜곡 — 렌즈와 이미지 센서가 완전히 평행하지 않아 발생

---

## 5. 차선 인식 처리 파이프라인

`lane_detector_node.py`의 `_image_cb()` 에서 프레임마다 다음 8단계가 실행된다.

```
원본 이미지
    │
    ▼ Step 1: 렌즈 왜곡 보정 (undistort)
    │
    ▼ Step 2: ROI 사다리꼴 정의
    │
    ▼ Step 3: 원근 변환 → 버드아이 뷰 (warpPerspective)
    │
    ▼ Step 4: 그레이스케일 + 가우시안 블러
    │
    ▼ Step 5: Canny 엣지 검출
    │
    ▼ Step 6: HoughLinesP 직선 검출
    │
    ▼ Step 7: 좌/우 차선 분리 + 직선 피팅
    │
    ▼ Step 8: 차선 중심 오차 계산 → /lane_error 퍼블리시
```

---

### Step 1: 렌즈 왜곡 보정

```python
frame = cv2.undistort(frame, self.K, self.D)
```

`/camera_info` 토픽에서 K (카메라 행렬), D (왜곡 계수)를 수신해 자동 적용된다.  
왜곡 보정 없이 차선 검출을 하면 직선 차선이 곡선으로 보여 오검출이 발생한다.

**보정 전 / 보정 후 비교:**
```
보정 전: 이미지 가장자리의 차선이 안쪽으로 휘어 보임
보정 후: 실제 직선 차선이 이미지에서도 직선으로 보임
```

---

### Step 2: ROI 사다리꼴 정의

카메라는 도로와 하늘(또는 본넷)을 동시에 찍는다. 차선 검출에 필요한 영역(**도로 부분**)만 잘라내기 위해 **사다리꼴(Trapezoid)** 형태의 관심 영역을 정의한다.

```
이미지 좌표계 (좌상단 = 원점)

(0,0)─────────────────────(1280,0)
  │                              │
  │    (tl_x, t_y)──(tr_x, t_y) │  ← roi_top_y (지평선 근처)
  │         /            \       │
  │        /              \      │
  │ (bl_x,b_y)────────(br_x,b_y)│  ← roi_bottom_y (본넷 윗부분)
  │                              │
(0,720)──────────────────(1280,720)
```

파라미터 6개:
| 파라미터 | 의미 | 기본값 |
|----------|------|--------|
| `roi_top_y` | 사다리꼴 윗변 y좌표 | 350 |
| `roi_top_left_x` | 윗변 왼쪽 x좌표 | 480 |
| `roi_top_right_x` | 윗변 오른쪽 x좌표 | 800 |
| `roi_bottom_y` | 사다리꼴 아랫변 y좌표 | 620 |
| `roi_bottom_left_x` | 아랫변 왼쪽 x좌표 | 50 |
| `roi_bottom_right_x` | 아랫변 오른쪽 x좌표 | 1230 |

> **기본값은 브라켓 없이 테스트 시 참고용이며, 카메라 장착 위치가 결정되면 반드시 재조정해야 한다.**

---

### Step 3: 원근 변환 (Inverse Perspective Mapping, IPM)

카메라로 찍은 도로는 **원근감** 때문에 차선이 소실점을 향해 좁아진다.  
이를 **버드아이 뷰(위에서 내려다보는 시점)**로 변환하면 차선이 평행한 직선이 되어 검출이 용이해진다.

```
원본 이미지 (원근감 있음)      버드아이 뷰 (위에서 본 것처럼)
    /차선\                         |차선|
   / 차선  \          →            |    |
  /___차선___\                     |차선|
```

```python
src = np.float32([[tl_x, t_y], [tr_x, t_y],    # 사다리꼴 4꼭짓점
                   [br_x, b_y], [bl_x, b_y]])
dst = np.float32([[100, 0], [w-100, 0],          # 직사각형으로 매핑
                   [w-100, h], [100, h]])
M = cv2.getPerspectiveTransform(src, dst)
warped = cv2.warpPerspective(frame, M, (w, h))
```

`M`은 4쌍의 대응점으로 계산된 3×3 투영 변환 행렬이다.  
ROI 사다리꼴이 실제 도로 평면을 정확히 잡을수록 버드아이 변환의 품질이 높아진다.

---

### Step 4: 그레이스케일 + 가우시안 블러

```python
gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5, 5), 0)
```

- **그레이스케일**: 컬러 정보 불필요 — 연산량 감소
- **가우시안 블러**: 노이즈 픽셀을 평탄화하여 Canny 오검출 억제. 커널 크기 `(5, 5)` = 5×5 픽셀 범위를 평균화.

---

### Step 5: Canny 엣지 검출

```python
edges = cv2.Canny(blur, canny_low, canny_high)
```

밝기 변화량(그래디언트)이 큰 경계선을 검출한다. 차선은 도로와 밝기 차이가 크므로 엣지로 검출된다.

**Hysteresis Thresholding:**
- 그래디언트 > `canny_high` → 확실한 엣지
- `canny_low` < 그래디언트 < `canny_high` → 확실한 엣지와 연결된 경우만 포함
- 그래디언트 < `canny_low` → 제거

| 파라미터 | 역할 | 기본값 |
|----------|------|--------|
| `canny_low` | 하위 임계값 | 50 |
| `canny_high` | 상위 임계값 | 150 |

> **트랙 환경에 따라 조정**: 조명이 밝으면 값을 높이고, 어두우면 낮춘다.

---

### Step 6: HoughLinesP 확률적 허프 직선 검출

```python
lines = cv2.HoughLinesP(
    edges, rho=1, theta=np.pi/180,
    threshold=hough_threshold,
    minLineLength=hough_min_length,
    maxLineGap=hough_max_gap)
```

엣지 이미지에서 직선 후보들을 검출한다.

**허프 변환 원리:**  
이미지 공간의 직선 `y = ax + b`를 파라미터 공간 `(a, b)`의 한 점으로 변환한다. 많은 엣지 점이 같은 파라미터를 공유하면 직선으로 판정된다.

| 파라미터 | 역할 | 기본값 |
|----------|------|--------|
| `rho` | 거리 해상도 (픽셀) | 1 |
| `theta` | 각도 해상도 (라디안) | π/180 (1°) |
| `hough_threshold` | 직선 인정 최소 투표 수 | 30 |
| `hough_min_length` | 직선으로 인정하는 최소 길이 (px) | 50 |
| `hough_max_gap` | 끊어진 선분 연결 허용 간격 (px) | 100 |

---

### Step 7: 좌/우 차선 분리 + 직선 피팅

```python
# 기울기와 위치로 좌/우 분류
slope = (y2 - y1) / (x2 - x1)
if slope < 0 and avg_x < mid_x:   # 좌측 차선
    left_pts.append(...)
elif slope > 0 and avg_x > mid_x: # 우측 차선
    right_pts.append(...)

# 1차 다항식 피팅: x = a*y + b
coeffs = np.polyfit(ys, xs, 1)
```

버드아이 뷰에서 차선 분류 기준:

```
버드아이 뷰
┌────────────────────────────┐
│  \ 좌측 차선  │  우측 차선 /│
│   \           │           / │
│    \          │          /  │
│     \         │(mid_x)  /   │
└────────────────────────────┘
  기울기 < 0, x < mid_x   기울기 > 0, x > mid_x
```

|abs(slope)| < 0.3인 선분은 수평선(도로 이음새 등)으로 판단해 제거한다.

---

### Step 8: 차선 중심 오차 계산

```python
# 이미지 맨 아래 행(y = h-1)에서 각 차선의 x좌표 계산
lx = left_fit[0] * (h-1) + left_fit[1]
rx = right_fit[0] * (h-1) + right_fit[1]

# 차선 중심
center = (lx + rx) / 2.0

# 정규화 오차 (-1.0 ~ +1.0)
error = (center - image_center_x) / image_center_x
```

**한쪽 차선만 검출된 경우 보정:**
- 좌측만 검출 → `center = lx + image_center_x * 0.4` (우측 차선이 오른쪽에 있다고 가정)
- 우측만 검출 → `center = rx - image_center_x * 0.4`
- 둘 다 미검출 → `error = 0.0` (오차 없음으로 유지)

---

### Step 8-b: 주행 방향 오차 계산 (Heading Angle)

횡방향 오차(`lateral_error`)만으로는 정확한 조향이 불가능하다.  
차선 내 위치가 맞더라도 **차체 각도가 틀리면** 곧 차선을 이탈한다.

```
lateral_error = 0  (위치는 중앙)    lateral_error = 0.3 (위치는 오른쪽)
      ↗ 차량 heading_error ≠ 0       → 차량 heading_error ≈ 0
──────────────────────────────        ──────────────────────────────
   차선 방향과 각도가 다름               위치가 틀려도 방향은 맞음
   → 즉시 차선 이탈                     → 자연히 돌아오는 중
```

**버드아이 뷰에서 heading 계산 원리:**

차선 피팅 모델: `x = a·y + b` (a = dx/dy)

```
a = 0        : 수직선 = 차량과 차선 방향 일치  → heading_error = 0°
a = +0.5     : 오른쪽 기울어짐 = 차체가 좌향  → heading_error = +26.6°
a = -0.5     : 왼쪽 기울어짐 = 차체가 우향   → heading_error = -26.6°

heading_deg = degrees(arctan(a))
```

```python
# _calc_error() 내부
avg_slope   = mean([left_fit[0], right_fit[0]])  # 있는 쪽만 평균
heading_deg = degrees(arctan(avg_slope))
```

**두 오차를 합친 조향 명령 (나중에 구현):**

```python
# 비례 게인은 튜닝 필요
steering = Kp_lat * lateral_error + Kp_hdg * (heading_deg / 45.0)
#                  ↑ 위치 보정              ↑ 각도 보정 (45° 기준 정규화)
```

**토픽:**
- `/lane_error`   — 횡방향 오차 (`Float32`, -1.0 ~ +1.0)
- `/lane_heading` — 주행 방향 오차 (`Float32`, 도(°), + = 차체가 좌향, - = 우향)

---

## 6. 카메라 방향 기준선 정렬

### 왜 이미지 중앙이 차량 방향이 아닌가

카메라를 차량에 장착할 때 완벽하게 정중앙에 맞추기는 현실적으로 어렵다.  
장착 오프셋이 있으면 아래 문제가 발생한다.

```
카메라가 오른쪽으로 1cm 치우친 경우:

    실제 트랙             카메라 이미지
    ─────────────         ┌──────────────┐
    |   차량   |          │  │(이미지중심)  │
    | 왼│ │오른|    →     │좌│  │  우차선  │
    | 차│ │차선|          │  │  │          │
    ─────────────         └──────────────┘
                              ↑ 차량은 중앙인데
                              이미지 중심이 왼쪽으로 치우침
                              → error ≠ 0 으로 오산
```

차량이 트랙 정중앙을 달리고 있어도 `lane_error != 0`이 출력되어 불필요한 조향 명령이 발생한다.

---

### 본넷 기준선을 이용한 일회성 정렬 방법

**목표:** 카메라 장착 완료 후, 이미지 중앙 = 차량 진행 방향이 되도록 물리적으로 정렬.

**준비물:** 흰 테이프 또는 마스킹 테이프, 자

**절차:**

**① 본넷에 임시 기준선 부착**

```
         카메라
           │
    ┌──────┼──────┐  ← 본넷 앞부분 (카메라에서 보이는 범위)
    │      │      │
    │    테이프   │  ← 본넷 중앙에 전후 방향으로 테이프 부착
    │      │      │
    └──────────────┘
```

차량의 좌우 폭을 줄자로 재서 정중앙을 찾고, 그 위치에 테이프를 앞뒤 방향으로 붙인다.  
테이프가 카메라 이미지에서 보일 만큼 충분히 길어야 한다 (10~15 cm 권장).

**② 시스템 실행 후 `/lane_debug` 확인**

```bash
ros2 launch gunsan_drive lane_detection.launch.py cam5_device:=/dev/video4
ros2 run rqt_image_view rqt_image_view   # 토픽: /lane_debug
```

`/lane_debug` 좌측 패널에 두 개의 세로선이 표시된다:
- **회색 선**: 이미지 픽셀 정중앙 (x = 640)
- **황색 선 (HEADING)**: 현재 조향 오차 계산 기준선

기본값에서는 두 선이 겹쳐있다.

**③ 테이프 선과 황색 기준선 일치 확인**

```
정렬 성공 (카메라가 정중앙에 있는 경우):
  ┌──────────────────────────┐
  │         |HEADING|        │
  │       테이프선            │
  │         |       |        │
  └──────────────────────────┘
  두 선이 겹침 → 완료

정렬 필요 (카메라가 오른쪽으로 치우친 경우):
  ┌──────────────────────────┐
  │      테이프선  |HEADING| │
  │           |      |       │
  └──────────────────────────┘
  테이프가 황색 선 왼쪽에 위치 → 카메라를 오른쪽으로 이동
```

**④ 카메라 위치를 조정하여 테이프 선 = 황색 선**

카메라 브라켓을 좌우로 조금씩 이동하며 이미지에서 테이프가 황색 기준선에 오도록 맞춘다.  
정렬이 완료되면 카메라를 브라켓에 단단히 고정한다.

**⑤ 테이프 제거**

정렬 후 테이프를 제거한다. 이후에는 이미지 중앙 = 차량 방향이 보장된다.

---

### 미세 조정: `heading_offset_x` 파라미터

물리적 정렬 후에도 잔여 오프셋이 있을 경우 소프트웨어로 보정한다.

**방법:** 차량을 트랙 정중앙에 직진으로 세운 뒤 `/lane_debug`를 확인한다.  
황색 기준선이 좌우 차선의 정가운데를 지나지 않으면 다음과 같이 조정한다.

```bash
# 기준선이 차선 중앙보다 오른쪽에 있을 때 (음수로 왼쪽으로 이동)
ros2 param set /lane_detector heading_offset_x -20

# 기준선이 차선 중앙보다 왼쪽에 있을 때 (양수로 오른쪽으로 이동)
ros2 param set /lane_detector heading_offset_x 20
```

`/lane_error`가 0에 가까워지면 적절한 offset 값을 찾은 것이다.  
확정된 값은 `lane_detection.launch.py`의 파라미터 기본값에 반영 후 리빌드한다.

> **물리 정렬을 정확히 하면 `heading_offset_x`는 0(기본값)으로 유지할 수 있다.**  
> 이 파라미터는 보정용 안전장치이다.

---

## 7. ROI 설정 방법

ROI는 **카메라 장착 위치**에 따라 완전히 달라진다.  
브라켓을 제작하고 카메라를 최종 위치에 고정한 뒤 아래 절차로 튜닝한다.

### 카메라 장착 위치와 ROI의 관계

```
카메라가 높이 장착 (위에서 내려다봄)
→ roi_top_y가 낮아짐 (지평선이 화면 위쪽에 위치)
→ roi_bottom_y는 본넷이 보이는 경계까지

카메라가 낮게 장착 (앞을 바라봄)
→ roi_top_y가 높아짐 (지평선이 화면 중간)
→ roi_bottom_y는 본넷 윗선 바로 위

카메라가 좌우로 기울어진 경우
→ roi_top_left_x, roi_top_right_x 비대칭 조정
```

### ROI 튜닝 절차

**준비:** 실행 중에 `/lane_debug` 이미지를 rqt로 실시간으로 보면서 조정한다.

```bash
# 터미널 1: 시스템 실행
source /opt/ros/humble/setup.bash
source ~/Gunsan_ws/install/setup.bash
ros2 launch gunsan_drive lane_detection.launch.py cam5_device:=/dev/video4

# 터미널 2: 디버그 이미지 뷰어
ros2 run rqt_image_view rqt_image_view
# 토픽 선택: /lane_debug

# 터미널 3: 파라미터 실시간 조정
source /opt/ros/humble/setup.bash
```

**Step A: 본넷 경계 찾기 (`roi_bottom_y`)**

본넷의 윗선이 이미지에서 몇 번째 픽셀 행인지 확인한다.

```bash
# /lane_debug 좌측 패널에서 녹색 사다리꼴 아랫변을 확인하며 조정
ros2 param set /lane_detector roi_bottom_y 600
```

아랫변이 본넷 위에 살짝 걸치는 위치에 맞춘다.

**Step B: 지평선 위치 찾기 (`roi_top_y`)**

도로가 하늘/벽과 만나는 지점(소실점 근처)까지만 포함한다.

```bash
ros2 param set /lane_detector roi_top_y 350
```

하늘/배경이 포함되면 노이즈 증가 → 값을 높여서 더 가까운 도로만 포함시킨다.

**Step C: 좌우 폭 맞추기**

사다리꼴 윗변은 소실점 근처이므로 좁고, 아랫변은 카메라 시야 전체에 맞게 넓힌다.

```bash
# 윗변: 차선이 보이는 소실점 근처 폭
ros2 param set /lane_detector roi_top_left_x 480
ros2 param set /lane_detector roi_top_right_x 800

# 아랫변: 차선이 보이는 최대 폭
ros2 param set /lane_detector roi_bottom_left_x 50
ros2 param set /lane_detector roi_bottom_right_x 1230
```

**Step D: 버드아이 뷰 확인**

`/lane_debug` 우측 패널에서 버드아이 뷰를 확인한다.  
**올바른 상태**: 차선 두 개가 위아래로 평행한 직선으로 보임  
**잘못된 상태**: 차선이 X자로 교차하거나 극도로 기울어짐 → src 사다리꼴 재조정

**Step E: 파라미터 확정 후 launch 파일에 반영**

튜닝이 완료되면 `lane_detection.launch.py`의 파라미터 기본값을 수정해 저장한다.

```python
# lane_detection.launch.py 안의 파라미터 블록
'roi_top_y':           350,   # ← 확정값으로 수정
'roi_top_left_x':      480,
...
```

그 후 리빌드:
```bash
cd ~/Gunsan_ws
colcon build --packages-select gunsan_drive
source install/setup.bash
```

---

## 8. 실행 방법

### 환경 소싱 (터미널마다 실행)

```bash
source /opt/ros/humble/setup.bash
source ~/Gunsan_ws/install/setup.bash
```

### 카메라만 테스트

```bash
ros2 launch gunsan_drive cameras.launch.py cam5_device:=/dev/video4
```

카메라가 정상 동작하는지 확인:
```bash
ros2 topic hz /camera5/image_raw    # 30 Hz 근처여야 함
ros2 topic echo /camera5/camera_info --once
```

### 차선 검출 전체 실행

```bash
ros2 launch gunsan_drive lane_detection.launch.py \
    cam5_device:=/dev/video4 \
    camera_topic:=/camera5/image_raw
```

### 조향 오차 확인

```bash
ros2 topic echo /lane_error
ros2 topic echo /lane_heading
```

### 조향 테스트 (차선 인식 + 조향 제어, 모터 구동 없음)

```bash
ros2 launch gunsan_drive steering_test.launch.py \
    cam5_device:=/dev/video4 \
    serial_port:=/dev/ttyACM0
```

---

## 8-1. 조향 테스트 구동 가이드

### 전체 데이터 흐름

```
카메라 이미지
    ↓
lane_detector_node
    ├── /lane_error   (-1.0 ~ +1.0, 횡방향 위치 오차)
    └── /lane_heading (degrees, 차선 기준 차체 기울기)
         ↓
steering_controller_node
    └── /steering/cmd (0~255, 0=우 / 128=중립 / 255=좌)
         ↓
motor_driver_node
    └── 시리얼 → Arduino
         └── 조향 DC모터 (P제어 폐루프, 퍼텐셔미터 피드백)
```

### 제어 법칙

```
correction = Kp_lat × lateral_error  −  Kp_hdg × (heading_deg / 45°)
steering_pos = clamp(128 + correction × 127,  0, 255)
```

| 상황 | lateral_error | heading_deg | correction | 조향 |
|------|--------------|-------------|------------|------|
| 차선 정중앙, 정렬 | 0 | 0 | 0 | 중립 (128) |
| 오른쪽 이탈 | +0.5 | 0 | +0.5 | 좌회전 (192) |
| 왼쪽 이탈 | −0.5 | 0 | −0.5 | 우회전 (64) |
| 위치 맞지만 좌향 | 0 | +20° | −0.22 | 우회전 (100) |

### 조향 방향 검증 (처음 실행 시)

```bash
# 터미널 1: 조향 명령 수동 퍼블리시
source /opt/ros/humble/setup.bash && source ~/Gunsan_ws/install/setup.bash

# 좌회전 (255)
ros2 topic pub --once /steering/cmd std_msgs/UInt8 "data: 255"
# 중립 (128)
ros2 topic pub --once /steering/cmd std_msgs/UInt8 "data: 128"
# 우회전 (0)
ros2 topic pub --once /steering/cmd std_msgs/UInt8 "data: 0"
```

실제 조향 방향이 반대로 동작하면:
```bash
ros2 param set /steering_controller steer_invert true
```

### 게인 튜닝 절차

**Step 1: Kp_hdg = 0 으로 시작 (횡방향만 먼저 튜닝)**

```bash
ros2 param set /steering_controller Kp_hdg 0.0
ros2 param set /steering_controller Kp_lat 0.5   # 작은 값부터
```

- 과조향(oscillation): Kp_lat 감소
- 반응 느림: Kp_lat 증가
- 적절한 값 확정 후 저장

**Step 2: Kp_hdg 추가 (헤딩 보정)**

```bash
ros2 param set /steering_controller Kp_hdg 0.3
```

- 커브 진입 시 미리 조향되면 적절
- 불안정해지면 Kp_hdg 감소

**Step 3: 확정 값을 launch 파일에 저장 후 리빌드**

```python
# steering_test.launch.py
'Kp_lat': 0.8,  # ← 확정값
'Kp_hdg': 0.3,
```

```bash
cd ~/Gunsan_ws && colcon build --packages-select gunsan_drive
source install/setup.bash
```

### 안전 동작

| 상황 | 동작 |
|------|------|
| 차선 미검출 0.5초 초과 | 자동으로 중립(128) |
| 노드 종료(Ctrl+C) | 중립(128) 퍼블리시 후 종료 |
| `steering_enable: false` | 중립(128) 고정 |

---

## 9. 파라미터 레퍼런스

### lane_detector 노드 전체 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `camera_topic` | string | `/camera5/image_raw` | 입력 이미지 토픽 |
| `heading_offset_x` | int | 0 | 차량 방향 기준선 오프셋 (px, 0=이미지 중앙) — 물리 정렬 후 잔여 보정용 |
| `lane_width_px` | int | 280 | 버드아이 뷰에서 차선 폭 (px) — 단일 차선 검출 시 반대쪽 차선 위치 추정에 사용 |
| `roi_top_y` | int | 350 | ROI 윗변 y좌표 |
| `roi_top_left_x` | int | 480 | ROI 윗변 왼쪽 x좌표 |
| `roi_top_right_x` | int | 800 | ROI 윗변 오른쪽 x좌표 |
| `roi_bottom_y` | int | 620 | ROI 아랫변 y좌표 |
| `roi_bottom_left_x` | int | 50 | ROI 아랫변 왼쪽 x좌표 |
| `roi_bottom_right_x` | int | 1230 | ROI 아랫변 오른쪽 x좌표 |
| `canny_low` | int | 50 | Canny 하위 임계값 |
| `canny_high` | int | 150 | Canny 상위 임계값 |
| `hough_threshold` | int | 30 | 허프 직선 투표 임계값 |
| `hough_min_length` | int | 50 | 허프 최소 선분 길이 (px) |
| `hough_max_gap` | int | 100 | 허프 선분 연결 허용 간격 (px) |

### cameras.launch.py 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `cam5_device` | `/dev/video4` | camera5 디바이스 경로 |
| `cam6_device` | `/dev/video6` | camera6 디바이스 경로 |

### lane_detection.launch.py 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `cam5_device` | `/dev/video4` | camera5 디바이스 경로 |
| `cam6_device` | `/dev/video6` | camera6 디바이스 경로 |
| `camera_topic` | `/camera5/image_raw` | 차선 검출에 사용할 카메라 토픽 |

### steering_controller 노드 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `Kp_lat` | float | 1.0 | 횡방향 비례 게인 (클수록 강하게 보정) |
| `Kp_hdg` | float | 0.5 | 헤딩 비례 게인 |
| `steer_invert` | bool | false | 조향 방향 반전 (실제 방향이 반대이면 true) |
| `steering_enable` | bool | true | false 시 중립(128) 고정 |
| `lost_timeout_s` | float | 0.5 | 차선 미검출 허용 시간(초), 초과 시 중립 |

### steering_test.launch.py 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `cam5_device` | `/dev/video4` | camera5 디바이스 경로 |
| `camera_topic` | `/camera5/image_raw` | 차선 검출에 사용할 카메라 토픽 |
| `serial_port` | `/dev/ttyACM0` | Arduino 시리얼 포트 |

### 시리얼 프로토콜 (motor_driver → Arduino)

```
프레임: 0xAA | LEN | CMD | PAYLOAD | CRC(XOR) | 0x55

CMD_STEER  (0x04): payload=[pos]  0=우측 끝 / 128=중립 / 255=좌측 끝
CMD_STOP   (0x03): no payload  — 후륜 모터 정지
CMD_MOTOR  (0x01): payload=[direction, speed]  direction: 0=전진, 1=후진
```

Arduino 조향 캘리브레이션값 (car_control.ino 기준):
- 좌측 한계: ADC 572 (pos=255)
- 중립: ADC 487 (pos=128)
- 우측 한계: ADC 398 (pos=0)

---

## 10. 향후 개발 계획

### 현재 완료
- [x] `usb_cam` 기반 카메라 2대 구동 launch
- [x] Camera5/6 캘리브레이션 YAML 등록
- [x] 차선 검출 노드 (언디스토션 → ROI → IPM → Canny → Hough → 오차)
- [x] 실시간 파라미터 튜닝 지원
- [x] Drive_ws 통합 (motor_driver 패키지 심링크)
- [x] 조향 제어 노드 (PD 제어, 안전 watchdog)
- [x] 조향 테스트 launch (모터 구동 없이 조향만)

### 진행 예정

| 단계 | 항목 | 비고 |
|------|------|------|
| 1 | 조향 방향 검증 + 게인 튜닝 | `steering_test.launch.py` 사용 |
| 2 | 카메라 브라켓 제작 후 ROI 최종 확정 | 물리적 작업 |
| 3 | RPLIDAR A1M8 연결 및 scan 토픽 검증 | SDK 설치 필요 |
| 4 | 후륜 모터 드라이버 장착 후 속도 제어 노드 | 하드웨어 준비 후 |
| 5 | 전체 자율주행 통합 테스트 | |

---

*최종 수정: 2026-06-01*
