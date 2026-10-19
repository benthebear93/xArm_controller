# xArm flange ChArUco / RealSense eye-to-hand calibration

외부에 고정한 RealSense RGB 카메라와 xArm7 베이스 사이의 변환을 구합니다.
ChArUco 보드를 플랜지에 단단하게 고정하고 로봇의 위치와 방향을 바꿔 촬영합니다.
보드의 정확한 플랜지 장착 위치를 미리 측정하지 않아도 됩니다. 프로그램이
카메라 위치와 보드 장착 위치를 함께 계산합니다.

프로그램: [`scripts/calibrate_eye_to_hand.py`](scripts/calibrate_eye_to_hand.py)

## 자동 이동·촬영·계산

[`scripts/auto_calibrate.py`](scripts/auto_calibrate.py)는 **실제로 로봇을 움직이는 자동 수집 프로그램**입니다.

기본 25개 자세는 **작은 범위의 초기 보정용**입니다. IK/FK 검사를 통과해도
화면 좌하단·우하단 분포나 카메라에 가까워지는 거리 변화까지 확보되는 것은 아닙니다.
특히 내부 파라미터까지 추정할 때 국소 자세만으로 최종 정확도를 판단하지 마세요.
아래의 영상 분포 검사로 촬영 후보를 확인하고 별도 자세로 결과를 검증해야 합니다.
먼저 TP/Studio로 보드가 카메라에 충분히 보이는 시작 자세를 잡고 조그를 해제합니다.
카메라와 로봇 베이스를 고정하고, 보드는 플랜지에 강체로 연결되어 있어야 합니다.
RealSense viewer를 종료한 뒤 실행합니다.

```bash
# 카메라 검출 + 현재 로봇 상태 + 전체 경로 검사만 수행. 모터 활성화/이동 없음.
.venv/bin/python scripts/auto_calibrate.py --dry-run \
  --session output/calibration/auto01

# 실제로 25개 자세를 이동·촬영한 뒤 calibration.json까지 계산.
.venv/bin/python scripts/auto_calibrate.py \
  --session output/calibration/auto01
```

자동 모드에서 `--dry-run`을 생략하면 **이동이 시작됩니다**. 추가 확인 프롬프트는 없습니다.
기본 dry-run은 `motion_plan.json`과 `initial_view.png`만 저장하므로 같은 폴더에서 실제 실행할 수 있습니다.
`--start-plan`으로 복귀가 필요한 경우에는 아래 설명처럼 복귀 경로만 검사합니다.
실행 시 현재 자세를 다시 읽어 경로를 새로 계산하며, 이전 계획 파일을 그대로 재생하지 않습니다.
이미 촬영한 session은 덮어쓰지 않습니다. 새 이름을 쓰거나 기존 데이터로 수동 `solve`를 실행합니다.

| 기본 설정 | 값 |
| --- | --- |
| 촬영 자세 | 시작 자세 포함 25개 |
| 목표 플랜지 위치 범위 | 시작점에서 반경 25 mm |
| 목표 플랜지 회전 범위 | 시작 방향에서 최대 12° |
| 관절 속도 / 가속도 | 3°/s / 10°/s² |
| 정지 확인 시간 | 자세마다 0.8초 |
| 관절 변화 상한 | 각 관절이 시작값에서 30° 이내 |

초기 플랜지의 X/Y 방향을 중심으로 기본 12°·7.2° 기울기를 만듭니다.
첫 5개 자세에 서로 반대인 X/Y 기울기를 넣어 최대 24°의 방향 차이를 먼저 확보합니다.
첫 번째 이동부터 위치도 바꾸며, 세 묶음은 지정 반경의 50/75/100%를 사용합니다.
초기 플랜지 X/Y/Z의 양·음 방향을 모두 포함합니다. 이 로컬 Z는 카메라 깊이 방향과 다릅니다.
TCP 오프셋이 있어도 회전·이동 기준은 **플랜지**이며, 보드 중심을 회전축으로 고정하는 기능은 아닙니다.
목표들이 시작 자세 기준으로 생성되므로 위치 이동이 누적되지 않습니다.

촬영 이동 전에는 인쇄 패턴의 전체 외곽이 화면 안에서 **최소 10 px** 여유를 갖는지 검사합니다.
**35 px는 권장값**이며, 10–35 px이면 안내를 출력하고 진행합니다. 검출 흔들림으로
34.8 px처럼 권장값에 조금 못 미쳐도 중단하지 않습니다. 추정 여유와 경계는 `initial_view.json`에 저장합니다.
내부 코너가 12개 이상 보여도 패턴 윗변이 잘린 시작 자세에서는 출발하지 않습니다.
시작 검사에 실패하면 저장된 `initial_view.png`를 보고 시작 자세를 조정하세요.
촬영 도중의 10 px 외곽 여유 검사는 **경고용**입니다. ChArUco는 일부 잘린 시점도 사용할 수 있으므로,
유효한 비공선 코너가 12개 이상이고 나머지 촬영 검사를 통과하면 저장을 계속합니다.
`full_pattern_margin_ok`와 `visibility_warning`을 각 샘플에 기록합니다.
이 평면 외곽 검사는 종이·브래킷이나 장애물까지 포함하는 3D 충돌 검사가 아닙니다.

정지 후 촬영은 기본 검출 외에 2배 확대 검출을 시도해 코너가 더 많이 검출되는 결과를 사용합니다.
코너 좌표는 원본 해상도로 환산하며 최소 12개, 재투영 오차와 정지 검사 기준은 유지합니다.
정지 상태에서 코너 수만 부족하면 새 프레임으로 최대 3회 시도합니다. 촬영 재시도 사이에는
150 ms 기다리고 로봇이 같은 자세인지 다시 검사하며, 저장 기록에 `detection_attempts`를 남깁니다.
잘못된 ID·로봇 이동·컨트롤러 오류는 재시도하지 않습니다. 정지 중 미리보기 갱신도 최대 3개
새 프레임을 확인합니다. 미리보기와 이동 중 보드 존재 감시는 유효한 비공선 코너 **6개**가
필요하며, 촬영 저장에는 여전히 **12개**가 필요합니다. 코너 ID 범위·중복·공선 검사는 양쪽에 적용합니다.
이동 중 1초 카메라/보드 추적 감시 시간은 그대로 유지하며, 마지막 검출 개수를 오류에 표시합니다.
세 프레임 모두 부족하면 다음 자세로 이동하지 않고 중단합니다.
이동 중 감시에는 확대 재검출을 적용하지 않습니다.

이동 전에 모든 목표의 IK, 관절 한계, FK 일치를 확인합니다. 관절 보간 경로도
최대 관절 간격 1°로 나누어 FK를 검사합니다. 중간 경로와 실제 피드백의 플랜지 제한은
위치 반경 `--radius-mm + 5 mm`, 회전각 `--tilt-deg + 3°`입니다.
목표 하나라도 검사에 실패하면 모터를 활성화하지 않습니다. 시작 자세 또는 범위를 조정해 다시 실행하세요.

**이 검사는 주변 장애물, 로봇 링크 전체, A4 보드의 회전 공간을 검사하는 충돌 계획기는 아닙니다.**
플랜지 위치가 고정돼 있어도 보드 모서리는 크게 움직일 수 있으므로 실제 작업공간에서 여유를 확보하세요.
계획 경로는 직선 TCP 이동이 아닌 저속 관절 이동입니다.

```bash
# 이동 범위를 줄인 예시
.venv/bin/python scripts/auto_calibrate.py --radius-mm 15 --tilt-deg 10 --speed 2 \
  --session output/calibration/auto02
```

허용 설정 범위는 위치 반경 5-50 mm, 기울기 10-20°, 관절 속도 최대 5°/s입니다.
`--serial`, `--width`, `--height`, `--intrinsics`, `--fit-intrinsics`도 지원합니다.
자동 프로그램은 공장 왜곡 모델을 OpenCV에서 직접 사용할 수 없으면, 촬영한 25개 영상으로
카메라 내부 파라미터도 추정하도록 **자동 전환**합니다. 별도 옵션 없이 같은 명령을 실행하면 됩니다.
`--fit-intrinsics`는 호환되는 공장 모델에서도 내부 파라미터를 새로 추정할 때 사용합니다.
이 모드에서는 촬영 중 공장 내부 파라미터로 재투영 오차를 검사하지 않고, 수집 후 추정한 값으로 검사합니다.
자동 선택 여부는 `motion_plan.json`과 `session.json`의 `fit_intrinsics`에 기록됩니다.
중단된 자동 수집을 오프라인으로 계산할 때 이 값이 `true`라면 수동 `solve` 명령에 `--fit-intrinsics`를 추가하세요.

- 영상 창에서 **Space / Q / Esc**, 또는 터미널 **Ctrl+C**로 정지를 요청합니다.
- 이동 중 로봇 상태와 새 영상을 폴링하고, 1초 넘게 영상이나 보드 검출이 끊기면 정지를 요청합니다.
  카메라 수신은 이동 중 블로킹하지 않습니다.
- 로봇 오류, 오프셋 변경, 이동 시간 초과, 촬영 실패 시 중단합니다. 오류를 자동으로 지우거나 재개하지 않습니다.
- 수집한 데이터는 매번 저장합니다. 중단 시 `session.json`의 `status`와 `abort_reason`을 확인할 수 있습니다.
  가능하면 중단 직후 영상과 검출 정보도 `abort_view.png` / `abort_view.json`에 보존합니다.
- 정상 종료하면 마지막 자세에서 정지한 뒤 연결을 해제하고 보정값을 계산합니다.
  시작 자세로 자동 복귀하지 않습니다. 실패할 때도 복귀 동작을 보내지 않습니다.
- 그래픽 화면이 없으면 `--headless`를 추가합니다. 이때 키보드 정지는 터미널 Ctrl+C를 사용합니다.

소프트웨어 정지는 통신 상태에 영향을 받으므로 실제 운전 중에는 물리 E-stop을 사용할 수 있어야 합니다.
결과가 계산되면 아래의 **새 자세 검증**으로 확인하세요. 자동 이동 코드의 테스트는 모의 장비를 사용하며,
실제 작업공간의 충돌 여유나 하드웨어 운전 성공을 확인한 것은 아닙니다.

### 중단된 수집 검토와 수정 자세 미리보기

```bash
.venv/bin/python scripts/review_calibration_session.py --session output/calibration/auto02
.venv/bin/python scripts/auto_calibrate.py --plan-only --radius-mm 50 --tilt-deg 12 \
  --session output/calibration/preview02
.venv/bin/python scripts/viser_robot.py --offline --play --port 8082 \
  --plan output/calibration/preview02/motion_plan.json
```

검토 결과는 `auto02/review/review.json`과 `frames_review.png`에 저장합니다.
검토 도구의 진단용 행렬 계산은 최종 보정이 아니며, 실행에 쓸 변환 행렬을 내보내지 않습니다.
2026-10-19의 auto02는 8장 이후 코너 부족으로 중단됐고, 최대 회전 변화 12° 및
최소 내부 보정 영상 수 12장 조건을 만족하지 못했습니다. 최종 `calibration.json`은 없습니다.
현재 수정된 50 mm 계획 역시 초기 보정용이며, 유효한 보정 결과 없이 화면 하단까지의 큰 이동을
실행하도록 바꾸지는 않았습니다. 시작 자세를 바꾸면 그 자세에서 IK/FK 검사를 다시 실행합니다.

## 카메라 영상으로 촬영 위치·거리 검토

### 앞으로 크게 뻗고 높이를 올리는 wide 모드

`--pattern wide`는 로봇 **베이스 +X로 220 mm, +Z로 최대 70.5 mm, Y로 ±80 mm**
움직이는 자세에 **Z −100 mm, Y ±150 mm의 좌하단·중앙 하단·우하단**을 추가합니다.
하단 세 위치에서 각각 세 기울기로 촬영하며 총 34개 촬영 자세입니다.
위치를 바꾸는 경유점은 40 mm 이하 간격으로 넣고, 도착 후 기울이도록 분리합니다.
기본 설정은 이동 경유점 포함 71개입니다. `--drop-mm`와 `--lower-side-mm`로 하단 폭을 조절하고,
`--drop-mm 0`이면 하단 구간을 생략해 이전 25개 촬영 자세를 유지합니다.
카메라 방향으로 보정된 좌표가 아니라 로봇 베이스 축입니다.

상단 잘림을 줄이기 위해 v3에서는 촬영 11–13번과 17–19번의 높이를 30 mm,
14–16번을 79.5 mm 낮췄습니다(기본 `--lift-mm 150` 기준). 이 9자세의 X/Y와 방향,
그 밖의 촬영 자세는 기존과 같습니다. `--lift-mm`는 기존 경로 범위 크기 설정을 유지하며,
현재 상단 목표는 그 값의 47% 높이를 사용합니다. 이전 저장 경로는 변경하지 않습니다.
`upper_pose_adjustment.json`에 이전 촬영 번호 기준 변경량을 기록했습니다.

```bash
# 실제 이동 없이 현재 자세에서 넓은 경로 검사 및 저장
.venv/bin/python scripts/auto_calibrate.py --plan-only --pattern wide \
  --forward-mm 220 --lift-mm 150 --side-mm 80 \
  --session output/calibration/preview_wide01

.venv/bin/python scripts/viser_robot.py --offline --play --port 8083 \
  --plan output/calibration/preview_wide01/motion_plan.json

# 카메라 시야/시작 자세도 검사. 이 명령도 이동하지 않음
.venv/bin/python scripts/auto_calibrate.py --dry-run --pattern wide --fit-intrinsics \
  --session output/calibration/auto_wide01
```

wide 모드의 관절 변화 제한은 시작점 대비 90°입니다(local은 30°).
허용 플랜지 경로는 시작점 기준 X [-20, forward+20], Y [−span−20, span+20],
Z [−drop−20, lift+20] mm의 상자와 시작 방향 대비 `tilt+3°`입니다.
span은 상단·하단 좌우 폭 중 큰 값입니다(하단을 끄면 side만 사용).
각 목표 및 관절 1° 간격 중간 경로를 컨트롤러 FK로 검사하며, 실패한 목표를 자동 생략하거나
범위를 몰래 줄이지 않습니다. 링크·보드·장애물의 충돌과 전체 경로의 카메라 가시성은 별도 확인 대상입니다.
시작 보드는 최소 10 px 여유가 필요하며 35 px를 권장합니다. 촬영 도중 10 px 여유 부족은 경고로 기록됩니다.
실제 수집은 위 `--dry-run` 명령에서 그 옵션을 제거하면 시작되므로,
높이 증가로 보드가 화면 위로 나가는지와 넓어진 작업 범위를 먼저 확인하세요.
하단 자세는 보드가 작업대에 더 가까워지므로, 플랜지 높이만으로 보드 하단의 여유를 판단하지 마세요.

좌우 하단이 들어간 저장 계획은 `output/calibration/preview_wide02/motion_plan.json`입니다.
Viser에서 `--start-pose 58`을 주면 이 계획의 하단 구간부터 볼 수 있습니다.
미리보기 시작 위치만 바꾸며 실제 수집 순서는 항상 시작 자세부터 전체 경로를 따릅니다.

### 고정 시작 자세로 복귀한 뒤 촬영

현재 선택한 시작 자세는 `auto_wide02/motion_plan.json`의 seed입니다.
TCP `[X, Y, Z, roll, pitch, yaw]`는
`[425.393494, -0.119884, 370.668274, -171.683537, -83.074755, -4.782421]` (mm, deg),
J1–J7은 `[-2.035662, 4.570656, -0.733902, 37.059426, -8.711938, -50.978748, 5.218156]` deg입니다.
이전 실행에서 시작 화면 검사를 통과했던 자세를 재사용하며, 복귀 후 가시성을 다시 확인합니다.

```bash
.venv/bin/python scripts/auto_calibrate.py --pattern wide --fit-intrinsics \
  --start-plan output/calibration/auto_wide02/motion_plan.json \
  --session output/calibration/auto_wide03
```

이 명령은 **실제 복귀 이동 후 새 캘리브레이션을 수행**합니다. 현재 자세와 저장된 경유점이
관절 0.5° / 플랜지 1 mm·0.5° 이내로 맞으면 그 지점부터 경로를 역순으로 따릅니다.
경유점 사이에서 멈췄다면 관절 공간 선분에 투영해 모든 관절의 차이가 0.1° 이내이고,
투영한 자세의 컨트롤러 FK와 현재 플랜지가 1 mm·0.5° 이내로 맞는지 확인합니다.
검사를 통과하면 선분의 이전 경유점으로 되돌아간 뒤 나머지 경로를 역순으로 따릅니다.
원래 관절 해와 TCP/월드 오프셋을 유지하고, 복귀 경로 전체의 관절 한계·FK·경로 범위를
모터 활성화 전에 재검사합니다. 카메라 감시와 정지 키도 복귀 중 동작합니다.
경로와 다른 위치에서는 임의의 직행 복귀를 하지 않고 오류를 냅니다.
`--start-plan`을 생략하면 기존처럼 현재 자세가 촬영 기준이 됩니다.

`--dry-run`을 추가하면 움직이지 않습니다. 복귀가 필요한 위치에서는 `return_plan.json`만
저장하고 종료하며, 아직 도달하지 않은 시작 화면과 새 촬영 경로까지 통과했다고 판단하지 않습니다.
이미 시작 자세라면 일반 촬영 dry-run을 수행합니다. `--plan-only`와는 함께 사용할 수 없습니다.
기존 `auto_wide02`의 이미지와 계획은 수정하지 않으며, 완료된 복귀를 제외한 수집 이어하기는 지원하지 않습니다.
링크·장애물·보드의 충돌은 모델링하지 않으므로, 저장 당시와 작업 공간이 달라졌다면 경로를 다시 확인해야 합니다.

### 화면 분포 후보 생성

로봇에는 연결하지 않고 D455 RGB 한 장과 내부 파라미터를 읽어 화면 분포를 검토합니다.

```bash
.venv/bin/python scripts/plan_camera_coverage.py --capture \
  --output output/calibration/coverage_review
```

`coverage_review.png`에는 화면의 3×3 위치를 현재 깊이의 100%, 85%, 70%에서 비교합니다.
가로 중심은 화면의 30/50/70%, 세로 중심은 29/50/71%이며 좌하단·우하단을 포함합니다.
각 위치에서 보드가 카메라를 향하도록 기본 방향을 잡은 뒤 서로 다른 기울기를 추가합니다.
초록 윤곽은 **인쇄 패턴** 전체가 화면 경계에서 35 px 이상 떨어진 후보이고,
빨강은 여백 검사에 실패한 후보입니다. 흰 종이 외곽·브래킷·케이블·장애물은 검사하지 않습니다.

`coverage_goals.json`은 **촬영 목표이며 로봇 이동 계획이 아닙니다**.
`T_camera_board`만으로 카메라 방향을 로봇 베이스 이동 방향으로 알 수는 없습니다.
초기 `calibration.json`이 있으면 다음 명령으로 `T_base_flange` 목표도 계산할 수 있습니다.
카메라 위치와 보드 장착이 초기 보정 때부터 유지되어야 합니다.

```bash
.venv/bin/python scripts/plan_camera_coverage.py \
  --image output/calibration/coverage_review/current.png \
  --camera output/calibration/coverage_review/camera.json \
  --calibration output/calibration/auto01/calibration.json \
  --output output/calibration/coverage_mapped
```

이 좌표 변환도 IK/FK·관절 제한·중간 경로·작업대 충돌 검사는 수행하지 않습니다.
`coverage_goals.json`을 `motion_plan.json`으로 바꾸거나 자동 이동기에 직접 넘기면 안 됩니다.
초기 화면에서 보드가 잘리면 우선 보드 전체가 보이는 시작 자세를 잡고 초기 데이터를 수집하세요.
화면 하단 목표는 시작점에서 수십 cm 떨어질 수 있으므로 실제 도달 범위에 맞게 계획해야 합니다.

2026-10-19 현장 영상에서는 40개 코너 중 12개만 보였고 보드 중심은 (615, 5) px,
추정 깊이는 0.844 m였습니다. 새 후보 깊이는 약 0.844/0.717/0.591 m입니다.
이는 공장 내부 파라미터와 일부 보드 코너로 추정한 값이며 실제 거리 측정이나 hand-eye 결과가 아닙니다.
D455의 inverse Brown 왜곡은 SDK 역투영으로 처리하며 OpenCV Brown 계수로 오해하지 않습니다.

참고: [OpenCV의 다양한 시점에서 ChArUco 영상 수집](https://docs.opencv.org/5.0/tutorials/objdetect/aruco_calibration/aruco_calibration.html),
[RealSense 투영·역투영과 왜곡 모델](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/).

## 1. 사용할 보드와 준비

패턴은 현재 폴더의 **`calib.io_CHARUCO_210x297_9x6_30_22_DICT_4X4.pdf`**입니다.
기본 설정의 치수는 2026-10-19 사용자가 측정한 **장착된 출력물** 기준입니다.
PDF 파일명에 적힌 설계 치수(30/22 mm)와 구분하세요.

| 항목 | 값 |
| --- | --- |
| 실제 가로 × 세로 칸 수 | 6 × 9 |
| 실측 체커 한 칸 | 27.3 mm |
| 실측 ArUco 마커 외곽 한 변 | 20.3 mm |
| OpenCV dictionary | `DICT_4X4_50` |
| 마커 ID | 0-26 |
| ChArUco 교차점 | 40개 |
| Legacy pattern | `false` |

PDF 패턴에는 마커 27개와 교차점 40개가 있습니다.
`assets/calibration/calib_io_charuco.json`에 실측값과 원본 설계값을 함께 기록했습니다.
`output/pdf/charuco_a4_30mm.pdf`는 마커 **22.5 mm**인 별도 보드이므로 혼용하지 마세요.

1. **100% / Actual Size**로 출력하고 실제 체커 한 변이 30 mm인지 측정합니다.
   출력 크기를 바꿨다면 설정 JSON을 복사해 체커와 마커의 실제 길이를 둘 다 수정하고
   `collect --board 수정한설정.json`을 사용합니다.
2. 보드를 평평하고 단단한 판에 붙여 플랜지에 고정합니다. 촬영 중 판이 휘거나
   플랜지에 대한 보드 위치가 변하면 보정할 수 없습니다. 나사나 브래킷으로 패턴을 가리지 않습니다.
3. 카메라와 로봇 베이스를 고정하고 보드 전체가 카메라에 보이도록 시작합니다.
4. Studio의 **world offset은 `[0,0,0,0,0,0]`**이어야 합니다.
   TCP offset은 현재 설정을 읽어 수학적으로 제거하므로 기존 값을 유지할 수 있습니다.
   프로그램은 오프셋이나 모터 상태를 변경하지 않습니다.
5. 기존 RealSense viewer를 종료해 카메라 점유를 해제합니다.

기존 세션은 촬영 당시 보드 설정을 내부에 저장하므로 기본 JSON만 고쳐서는 다시 계산되지 않습니다.
실측값을 반영해 원본 영상에서 다시 검출하고, 별도 세션으로 계산하려면:

```bash
.venv/bin/python scripts/reprocess_calibration.py \
  --session output/calibration/auto_wide04 \
  --output output/calibration/auto_wide04_measured
.venv/bin/python scripts/calibrate_eye_to_hand.py solve \
  --session output/calibration/auto_wide04_measured --fit-intrinsics
```

재처리 출력 폴더는 새 경로여야 합니다. 원본 영상·로봇 기록을 보존하고 새 검출 결과 및
탈락 이유를 `reprocessing.json`에 남깁니다. 치수 변경 전의 보드 자세/재투영 결과는 재사용하지 않습니다.
`auto_wide04_measured`는 이미 생성됐으며 31장 모두 재검출을 통과했습니다.
기존 Park 계산의 위치 RMS는 12.018 → 4.240 mm로 줄었지만 아직 3 mm 기준에는 미달합니다.

필요 패키지는 기존 `requirements.txt`와 `requirements-realsense.txt`에 있습니다.
현재 프로젝트 `.venv`에서 실행 및 테스트를 확인했습니다. 새 환경이면 README의 SDK 설치 후:

```bash
uv pip install --python .venv/bin/python -r requirements-realsense.txt
```

## 2. 로봇 자세와 영상 수집

그래픽 데스크톱 터미널에서 실행합니다.

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py collect \
  --ip 192.168.1.197 \
  --session output/calibration/run01
```

RGB 기본 해상도는 1280 × 720, 30 FPS입니다. 카메라에서 지원하지 않으면
`--width 640 --height 480`을 추가합니다. 카메라가 여러 개라면 `--serial 카메라시리얼`을 지정합니다.

- 로봇 이동은 UFACTORY Studio 또는 기존 `scripts/teach_pendant.py`로 합니다.
  이 프로그램은 **로봇 상태와 자세를 읽는 기능만** 사용합니다.
- 로봇을 정지시킨 다음 **영상 창에 포커스를 두고 Space 또는 S**를 누릅니다.
- 프로그램이 0.6초 동안 정지 여부를 확인하고 새 RGB 프레임을 읽은 뒤 포즈를 다시 확인합니다.
  이는 정지 상태의 짝지어진 촬영이며 이동 중 연속 촬영용 시간 동기화는 아닙니다.
- **20-30개의 서로 다른 자세**를 저장합니다. 최소 계산 개수는 8개입니다.
  단순히 좌우 이동만 하지 말고, 서로 다른 두 축 이상으로 보드를 기울여 주세요.
  좌우/상하 기울기 ±15-30° 정도를 포함하고 거리와 화면 내 위치도 바꿉니다.
  움직임 범위는 실제 보드·브래킷과 주변 공간에 맞게 정합니다.
- 교차점이 최소 12개 보여야 합니다. 많은 교차점이 넓게 보이는 구도가 좋습니다.
  자동 내부 파라미터 추정도 할 계획이라면 화면 중앙과 가장자리, 여러 거리에서 수집합니다.
- **Q 또는 Esc**로 종료합니다. 성공한 촬영은 매번 저장됩니다.

움직임(0.3 mm / 0.1° 초과, 관절 0.1° 초과), 중복 포즈(5 mm와 3° 둘 다 미만),
검출 부족, 지원되는 내부 파라미터 사용 시 재투영 RMS 2 px 초과를 거부합니다.
거부 사유는 터미널에 출력됩니다. 재투영 오차는 검출된 교차점과 추정 보드 포즈로
다시 투영한 교차점 사이의 픽셀 거리입니다.

```text
output/calibration/run01/
  session.json       보드/카메라 설정, 로봇 포즈, TCP offset, 검출점, 촬영 시각
  images/0001.png    각 샘플의 원본 RGB 영상
  images/0002.png
  ...
```

카메라·로봇 베이스·보드 장착이 그대로인 경우에만 이어서 수집합니다.

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py collect \
  --session output/calibration/run01 --resume
```

카메라나 보드 장착을 바꿨다면 새 session으로 시작합니다.
해상도, 카메라 시리얼, 보드 설정 등이 다르면 기존 session에 추가할 수 없습니다.

## 3. 보정 계산

로봇이나 카메라 연결 없이 저장된 데이터만으로 실행합니다.

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py solve \
  --session output/calibration/run01
```

기본값은 RealSense의 RGB 공장 내부 파라미터와 Park hand-eye 계산입니다.
현재 설치된 OpenCV 5 wheel처럼 `calibrateHandEye` Python 함수가 없는 환경에서는
NumPy로 구현한 Park 회전 로그/SVD 및 병진 최소제곱 계산을 사용합니다.

`calibration.json`에 변환 행렬, 샘플별 오차, 제외/거부 사유와 전체 오차가 저장됩니다.
기본 품질 기준은 보드 포즈 일치 오차의 **병진 RMS ≤ 3 mm, 회전 RMS ≤ 1°**입니다.
이 값은 데이터 일관성 검사 기준이며 실제 작업 정확도를 보증하는 값은 아닙니다.
기준을 초과해도 진단용 JSON은 저장하지만 `passed_training_residual_limits`가 `false`이고
명령은 실패 상태로 종료합니다. 기준을 통과한 뒤에도 아래의 새 자세 검증을 수행하세요.

샘플별 오차와 원본 영상을 확인해 잘못 촬영한 데이터는 ID를 지정해 제외할 수 있습니다.
자동으로 큰 hand-eye 오차를 숨기거나 데이터를 삭제하지 않습니다.

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py solve \
  --session output/calibration/run01 --exclude 4 12
```

### 카메라 내부 파라미터 선택

RealSense `brown_conrady`, `none`, 또는 계수가 모두 0인 모델을 OpenCV에서 직접 사용합니다.
0이 아닌 `inverse_brown_conrady`나 `modified_brown_conrady` 계수를 OpenCV 계수로
잘못 해석하지 않습니다. 이 경우 촬영은 가능하지만 포즈/축 미리보기가 비활성화됩니다.
화면과 터미널 안내에 따라 충분히 다양한 자세를 수집한 뒤 다음처럼 계산합니다.

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py solve \
  --session output/calibration/run01 --fit-intrinsics
```

이 옵션은 최소 12개 영상의 ChArUco 교차점으로 OpenCV 카메라 내부 파라미터를 먼저 추정합니다.
20-30개 이상의 선명한 영상에서 다양한 기울기, 거리, 화면 위치를 포함하세요.
추정한 내부 파라미터와 카메라 보정 RMS도 결과의 `camera` 항목에 기록됩니다.
카메라를 별도로 보정한 값이 있다면 `--intrinsics camera.json`을 `collect` 또는 `solve`에 지정합니다.
형식은 다음과 같으며 수치는 해당 카메라에서 측정한 값으로 바꿔야 합니다.

```json
{
  "image_size": [1280, 720],
  "matrix": [[900, 0, 640], [0, 900, 360], [0, 0, 1]],
  "dist_coeffs": [0, 0, 0, 0, 0],
  "distortion_model": "opencv"
}
```

## 4. 새 자세로 결과 검증

카메라와 보드 장착을 유지하고 학습 때와 다른 자세 5-10개를 별도 session에 촬영합니다.
검증은 저장된 카메라 변환과 장착 변환을 그대로 사용하며 다시 피팅하지 않습니다.

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py collect \
  --session output/calibration/verify01

.venv/bin/python scripts/calibrate_eye_to_hand.py verify \
  --session output/calibration/verify01 \
  --calibration output/calibration/run01/calibration.json
```

각 자세의 mm/° 오차와 전체 RMS/max가 출력됩니다. 오차가 크다면 출력 크기,
판의 평탄도, 플랜지 고정 상태, 흔들림/흐림, 회전 다양성, 내부 파라미터를 확인합니다.

## 5. 결과 좌표계 사용

모든 4×4 변환의 병진 단위는 **미터**입니다. 원본 SDK 포즈는 별도 필드에 mm/°로 저장합니다.

| 결과 키 | 의미 |
| --- | --- |
| `T_base_camera` | RealSense **RGB optical** 좌표 → 로봇 물리 베이스 좌표 |
| `T_camera_base` | 베이스 좌표 → RGB optical 좌표 |
| `T_flange_board` | 보드 좌표 → 플랜지 좌표; 함께 추정한 보드 장착 위치 |
| `T_board_flange` | 플랜지 좌표 → 보드 좌표 |

보드 원점은 인쇄 패턴의 **왼쪽 위 외곽 모서리**입니다. 첫 내부 교차점이나 종이 모서리가 아닙니다.
보드 X축은 오른쪽, Y축은 아래, Z축은 인쇄면 안쪽입니다.
RGB optical 좌표계는 X 오른쪽, Y 아래, Z 카메라 앞쪽입니다.

```text
T_base_flange × T_flange_board = T_base_camera × T_camera_board
```

`T_a_b`는 b 좌표에서 a 좌표로 변환하는 행렬입니다. `flange`는 TCP 오프셋을 제거한
플랜지 프레임, `board`는 타깃 프레임, `camera`는 RGB optical 프레임입니다.
위 식의 양쪽으로 구한 보드 포즈가 얼마나 일치하는지 결과의 `residuals`로 보고합니다.

```python
import json
import numpy as np

with open("output/calibration/run01/calibration.json") as f:
    calibration = json.load(f)

if not calibration["quality"]["passed_training_residual_limits"]:
    raise ValueError("보정 품질 기준을 통과하지 못한 결과입니다.")

T = np.asarray(calibration["T_base_camera"])
p_camera_m = np.array([0.02, -0.01, 0.60])  # RGB optical frame의 3D 점, 단위 m
p_base_mm = 1000 * (T[:3, :3] @ p_camera_m + T[:3, 3])
print(p_base_mm)
```

이 행렬은 2D 픽셀을 직접 3D로 바꾸지 않습니다. 깊이 또는 알려진 평면 정보가 필요합니다.
RealSense depth optical 프레임의 점은 센서의 depth→color extrinsics로 RGB optical
프레임으로 변환한 뒤 이 행렬을 적용합니다. RGB와 depth의 원점을 같은 것으로 취급하지 마세요.

## 오프라인 보드 확인과 테스트

`tests/`는 로컬 개발용으로만 유지하며 배포 저장소에는 포함하지 않습니다.
아래 테스트 명령은 해당 디렉터리가 있는 개발 환경에서만 실행합니다.

```bash
pdftoppm -scale-to 1400 -singlefile -png \
  calib.io_CHARUCO_210x297_9x6_30_22_DICT_4X4.pdf /tmp/charuco

.venv/bin/python scripts/calibrate_eye_to_hand.py check-board \
  --image /tmp/charuco.png --output /tmp/charuco_detected.png

PYTHONPATH=scripts .venv/bin/python -m unittest discover -s tests -v
```

기하학 테스트는 알려진 카메라/장착 변환, 영상 투영 노이즈, TCP 회전·병진 오프셋,
퇴화 자세, 잘못 짝지어진 샘플, 새 자세 검증을 포함합니다. 하드웨어 mock은
수집 과정에서 읽기 API만 호출하는지 확인합니다. 실제 하드웨어 연결과 캘리브레이션
정확도는 이 오프라인 테스트로 확인되지 않습니다.

참고: [OpenCV hand-eye calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html),
[UFACTORY RPY 좌표계](https://docs.ufactory.cc/user_manual/ufactoryStudio/4.live_control.html),
[RealSense projection 및 왜곡 모델](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/).
