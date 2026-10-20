# xArm7 / RealSense eye-to-hand 보정

외부에 고정한 RealSense RGB 카메라와 로봇 베이스 사이의 변환을 구합니다.
ChArUco 보드를 플랜지에 고정해 여러 자세에서 촬영하면 카메라 위치와 보드 장착 위치를
함께 추정합니다. 설치는 [README](README.md#install)를 참고하세요.

## 준비

기본 패턴은 [calib.io PDF](assets/calib.io_CHARUCO_210x297_9x6_30_22_DICT_4X4.pdf),
설정은 [`calib_io_charuco.json`](assets/calibration/calib_io_charuco.json)입니다.

| 항목 | 기본 설정 |
| --- | --- |
| 가로 × 세로 칸 수 | 6 × 9 |
| 체커 / 마커 외곽 한 변 | 27.3 / 20.3 mm (장착된 출력물 실측값) |
| PDF 설계 치수 | 30 / 22 mm |
| Dictionary / 마커 ID | `DICT_4X4_50` / 0–26 |
| 교차점 / Legacy pattern | 40개 / `false` |

**100% / Actual Size로 출력한 뒤 체커와 마커를 모두 측정하세요.** 기본 설정은 특정
출력물의 실측값입니다. 다른 출력물에는 JSON을 복사해 치수를 수정하고 `--board PATH`를
지정합니다. 평평하고 단단한 판에 보드를 붙이고 플랜지에 고정합니다.
카메라·로봇 베이스·보드 장착은 수집과 검증 동안 유지해야 합니다.

Studio의 world offset은 `[0,0,0,0,0,0]`이어야 합니다. 기존 TCP offset은 읽어서
수학적으로 제거합니다. 프로그램은 오프셋을 변경하지 않습니다.
카메라를 점유하는 다른 프로그램을 종료하고, 보드 전체가 보이는 시작 자세를 잡습니다.

별도 패턴이 필요하면 `assets/calibration/generate_charuco.py`를 사용합니다.
OpenCV·NumPy 외에 ReportLab이 필요하며 `output/pdf/`에 PDF와 JSON을 생성합니다.
이 패턴은 30 / **22.5 mm**이므로 기본 calib.io 보드와 혼용하지 마세요.
장착 참고 도면: [플랜지](assets/calibration/xarm13xx_end_flange.png).

## 자동 이동·촬영·계산

```bash
# 기본: 이동 없이 검사
.venv/bin/python scripts/auto_calibrate.py

# 검사 후 실제 이동·촬영·계산
.venv/bin/python scripts/auto_calibrate.py --execute
```

기본값은 wide 패턴, 고정된 공장 내부 파라미터, 공동 최적화입니다.
`output/calibration`에서 같은 로봇 IP의 실제 촬영 또는 이동 기록이 있는 최신 세션을
찾아 저장 경로를 역순으로 따라 시작 자세로 복귀합니다. 미리보기/dry-run 계획은
자동 선택하지 않습니다. 기록이 없으면 현재 자세에서 시작합니다.
선택한 경로와 새 `auto_날짜_시간_마이크로초` 폴더를 터미널에 출력합니다.

| 옵션 | 의미 |
| --- | --- |
| `--current-pose` | 이전 경로 복귀 없이 현재 자세에서 시작 |
| `--start-plan PATH` | 복귀에 사용할 `motion_plan.json` 지정 |
| `--session PATH` | 새 출력 폴더 지정 |
| `--pattern local` | 작은 범위의 25개 자세 사용 |
| `--plan-only` | 현재 자세에서 IK/FK와 경로만 검사; 카메라/이동 없음 |
| `--headless` | 영상 창 없이 실행 |
| `--ip`, `--serial` | 로봇 IP / 카메라 시리얼 지정 |
| `--width`, `--height`, `--fps` | RGB 스트림 설정 (기본 1280 × 720, 30 FPS) |

`--execute`, `--dry-run`, `--plan-only`는 함께 사용할 수 없습니다.
**`--execute`는 검사 후 추가 확인 없이 이동합니다.** 복귀가 필요한 dry-run은 복귀 경로만
검사하며 새 촬영 경로는 실제 복귀 후 검사합니다. 현재 자세가 저장 경로와 맞지 않으면
복귀를 거부합니다. `--start-plan`은 `--current-pose` 또는 `--plan-only`와 함께 쓸 수 없습니다.
이미 촬영한 세션은 덮어쓰거나 자동 이어하기를 하지 않습니다.

| 패턴 | 촬영 위치와 방향 | 관절 변화 상한 |
| --- | --- | --- |
| wide (기본) | 34개; 베이스 +X 220 mm, 상단 +Z 70.5 mm, Y ±80 mm; 하단 Z −100 mm, Y ±150 mm; 기울기 12° | 시작 대비 90° |
| local | 25개; 시작 플랜지 기준 반경 25 mm, 기울기 12° | 시작 대비 30° |

Wide의 큰 이동은 위치 이동과 기울기를 분리한 경유점을 사용합니다.
`--forward-mm`, `--lift-mm`, `--side-mm`, `--drop-mm`, `--lower-side-mm`으로 범위를 바꿉니다.
상단 촬영 높이는 `--lift-mm`의 47%입니다. `--drop-mm 0`은 하단 9개를 생략합니다.
Local은 `--radius-mm` 5–50 mm, `--tilt-deg` 10–20°로 설정합니다.
기본 속도/가속도는 3°/s / 10°/s², 정지 확인 시간은 0.8초입니다.

이동 전 IK, 관절 한계, 목표 FK, 1° 간격의 관절 보간 경로를 검사합니다.
Local 경로는 반경 +5 mm / 기울기 +3°, wide 경로는 설정한 베이스 범위 +20 mm /
기울기 +3° 이내인지 검사합니다. **장애물·전체 로봇 링크·보드 회전 공간의 충돌 검사는
수행하지 않습니다.** 베이스 축이나 플랜지 축은 카메라 축과 같지 않습니다.

시작 화면은 패턴 전체 외곽에 최소 10 px 여유가 필요합니다(권장 35 px).
이후 일부 잘린 촬영은 유효한 비공선 교차점 12개와 나머지 촬영 기준을 만족하면
저장하고 외곽 경고를 기록합니다. 정지 중 코너 부족은 새 프레임으로 최대 3회 재시도합니다.
이동 중 보드 감시에는 6개 코너가 필요하며, 영상/보드 추적이 1초 이상 끊기면 중단합니다.

영상 창의 **Space / Q / Esc**, 터미널 **Ctrl+C**로 정지를 요청합니다.
로봇 오류, 오프셋 변경, 이동 시간 초과, 촬영 실패도 중단 사유입니다.
오류를 자동 해제하거나 재개하지 않습니다. 정상 종료와 실패 모두 자동 복귀하지 않습니다.
소프트웨어 정지는 통신에 의존하므로 물리 E-stop을 사용할 수 있어야 합니다.
수집은 매번 저장하며 중단 사유는 `session.json`, 가능하면 `abort_view.png/json`에도 남깁니다.

## 수동 수집

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py collect --session output/calibration/run01
```

로봇 연결은 읽기 전용입니다. Studio/teach pendant로 이동한 뒤 정지하고,
영상 창에서 **Space / S**로 촬영합니다. **Q / Esc**로 종료합니다.
20–30개 자세를 권장하며, 위치·거리·화면 분포와 서로 다른 두 회전축을 포함하세요.
최소 계산 개수는 8개입니다. 수집 시 다음을 거부합니다.

- 정지 확인 중 0.3 mm / 0.1° 초과 이동 또는 관절 0.1° 초과 변화.
- 이전 촬영과 위치 5 mm 및 방향 3° 차이가 모두 미만인 중복 자세.
- 유효 교차점 12개 미만, 잘못된 ID/공선 코너, 재투영 RMS 2 px 초과.

`session.json`은 보드/카메라 설정, 원본 로봇 포즈와 오프셋, 검출점, 시각을 저장하고,
`images/0001.png`부터 원본 RGB를 저장합니다. 촬영은 정지 상태에서 짝지은 기록이며
이동 중 연속 촬영의 시간 동기화가 아닙니다.
카메라·장착·설정이 그대로인 경우에만 같은 명령에 `--resume`을 추가해 이어서 수집합니다.

## 보정 계산

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py solve --session output/calibration/run01
```

하드웨어 연결 없이 저장된 데이터로 계산합니다. 기본 `--method joint`는 Park 초기값에서
카메라/베이스와 보드/플랜지의 두 강체 변환을 공동 최적화합니다.
내부 파라미터와 평면 보드 치수는 고정합니다. `--method park`는 순차 계산입니다.
Inverse Brown 왜곡은 RealSense SDK 역투영으로 처리하고 원래 모델과 계수를 보존합니다.

필요한 경우 `--intrinsics camera.json`으로 별도 측정값을 지정하거나,
`--fit-intrinsics`로 내부 파라미터를 먼저 재추정합니다(서로 함께 사용 불가).
재추정에는 최소 12개 영상이 필요하며, 다양한 거리와 화면 위치에서 더 많은 영상을
수집해야 합니다. 자동 수집에서 이 옵션을 쓰면 재투영 검사는 내부 파라미터 추정 후 수행합니다.
별도 내부 파라미터 JSON 예시이며 수치는 실제 측정값으로 바꾸세요.

```json
{
  "image_size": [1280, 720],
  "matrix": [[900, 0, 640], [0, 900, 360], [0, 0, 1]],
  "dist_coeffs": [0, 0, 0, 0, 0],
  "distortion_model": "opencv"
}
```

결과는 `calibration.json`이며 `--output PATH`로 변경합니다.
기본 품질 기준은 병진 RMS ≤ 3 mm, 회전 RMS ≤ 1°입니다.
실패해도 진단 JSON을 저장하지만 `quality.passed_training_residual_limits`는 `false`이고
명령은 실패 상태로 종료합니다. `per_sample`을 검토하고 필요하면 `--exclude 4 12`처럼
샘플 ID를 명시적으로 제외합니다. 학습 잔차는 절대 정확도가 아니므로 새 촬영으로 검증하세요.

## 새 자세 검증

```bash
.venv/bin/python scripts/auto_verify.py             # 이동 없이 검사
.venv/bin/python scripts/auto_verify.py --execute   # 새 자세 촬영 후 검증
```

같은 로봇의 품질 기준을 통과한 최신 보정값을 선택합니다. 특정 파일은 `--calibration PATH`로
지정합니다. 기본은 현재 자세에서 20 mm 떨어진 10개 위치, 8° 기울기이며 자동 복귀하지 않습니다.
`--pattern wide`나 명시적인 `--start-plan PATH`도 지원합니다.
구형 펌웨어의 local 검증은 이전 관절각에서 시작하는 FK 기반 수치 IK를 사용하며
관절 한계·경로·피드백 검사는 동일하게 적용합니다.

모든 촬영 목표와 측정 자세는 학습 자세와 위치 10 mm 및 방향 5° 차이가 모두 미만이면
거부됩니다. 이 검사를 위해 원본 학습 `session.json`도 필요합니다.
카메라/장착을 유지하며 내부 파라미터와 변환을 다시 추정하지 않습니다.
새 `verify_날짜_시간_마이크로초/`에 기준 사본 `reference_calibration.json`, 촬영 세션,
`verification.json`을 저장합니다. 결과는 해당 영역의 일관성 검사입니다.

수동 검증은 다른 자세 5–10개를 새 세션으로 촬영한 뒤 실행합니다.

```bash
.venv/bin/python scripts/calibrate_eye_to_hand.py collect --session output/calibration/verify01
.venv/bin/python scripts/calibrate_eye_to_hand.py verify \
  --session output/calibration/verify01 --calibration output/calibration/run01/calibration.json
```

수동 검증에서는 사용자가 학습과 다른 자세를 선택해야 합니다. 최소 3개 유효 자세가
필요합니다. 오차가 크면 출력 치수, 평탄도, 장착 고정, 영상 흐림, 회전 다양성을 점검하세요.

## 촬영 검토와 재처리

```bash
# 저장된 영상과 자세 분포 검토
.venv/bin/python scripts/review_calibration_session.py --session output/calibration/run01

# 현재 RGB에서 화면 분포 후보 생성 (로봇 이동 없음)
.venv/bin/python scripts/plan_camera_coverage.py --capture --output output/calibration/coverage

# 보드 치수가 바뀌면 원본 영상을 새 세션으로 재검출
.venv/bin/python scripts/reprocess_calibration.py --session output/calibration/run01 \
  --board assets/calibration/calib_io_charuco.json --output output/calibration/reprocessed
.venv/bin/python scripts/calibrate_eye_to_hand.py solve --session output/calibration/reprocessed
```

분포 후보는 카메라 공간의 촬영 목표이며 로봇 경로가 아닙니다. `--calibration PATH`를
추가하면 초기 보정값으로 로봇 좌표 목표도 계산하지만 실제 경로 검사는 별도로 필요합니다.
재처리는 원본을 보존하고 새 검출 결과와 거부 사유를 기록합니다.
기본 보드 JSON만 수정해서는 기존 세션 내부의 보드 치수가 바뀌지 않습니다.
보드 PNG를 오프라인으로 확인하려면 `calibrate_eye_to_hand.py check-board --image PATH`를 사용합니다.

## 결과 좌표계

4×4 변환의 병진 단위는 **미터**입니다. 원본 SDK 포즈는 mm/°로 별도 저장합니다.

| 키 | 변환 방향 |
| --- | --- |
| `T_base_camera` / `T_camera_base` | RGB optical → 물리 베이스 / 역변환 |
| `T_flange_board` / `T_board_flange` | 보드 → 플랜지 / 역변환 |

보드 원점은 패턴 왼쪽 위 **외곽 모서리**이며 X는 오른쪽, Y는 아래, Z는 인쇄면 안쪽입니다.
RGB optical은 X 오른쪽, Y 아래, Z 카메라 앞쪽입니다.

```text
T_base_flange × T_flange_board = T_base_camera × T_camera_board
```

`T_a_b`는 b에서 a로 좌표를 옮깁니다. `flange`는 TCP 오프셋을 제거한 플랜지,
`board`는 타깃, `camera`는 RGB optical 프레임입니다. `residuals`는 두 경로의 일치 오차입니다.
RGB optical의 3D 점에 `T_base_camera`를 적용하면 베이스 좌표를 얻습니다.
2D 픽셀에는 깊이 또는 알려진 평면 정보가 추가로 필요합니다.
Depth optical 점은 먼저 센서의 depth→color extrinsics로 RGB optical 좌표로 옮겨야 합니다.
