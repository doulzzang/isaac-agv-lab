# 2026-09-17 Spark Isaac Sim 5.1 검증 및 게시 대기본

실제 DGX Spark ARM64 / Isaac Sim 5.1에서 검증했습니다. Git 커밋·푸시, GitHub 릴리스 교체, 홈페이지 게시는 사용자 승인 대기입니다.

## 반영 사항

- Worker 안전 관찰 20초, 매초 상태 표시, 이동 범위 확대와 경로 충돌 검사.
- Worker 원본 신발 하단 약 7.3cm 오프셋 보정. 생성·복원·크기 변경 시 발을 바닥 기준에 정렬.
- 카메라 바닥 기준 높이 1.5m → 0.75m.
- 후진 목표 방향 오류 수정, 한국어 동의 표현 및 거리 우선 어순 처리.
- 차체 전체 여유 폭을 사용하는 선제 우회, 대상 접근점 선택과 충돌 체험 후 경계 탈출.
- ROS 2 Jazzy/Nav2 설치, 현재 DWB 설정 호환, USD 장애물 경계 pointcloud를 충돌 감시기에 연결.
- 방·전체 공간 교체 / 창고 화면 / 선택 객체 외형 메뉴를 브라우저에서 숨기고 불필요한 설명 정리.
- 제목: Isaac Sim 물류창고 AGV Robot Agentic 체험(광주과학기술 AI대학원)
- 운영 PC 선택·접속·조작권 transport 보존.

## 실제 자동체험 (동일 최종 코드 연속 실행)

| 체험 | 결과 | 실제 결과 |
|---|---|---|
| drive | PASS | 체험 자동실행 완료: 오른쪽으로 약 45도 회전했습니다. |
| approach | PASS | 체험 자동실행 완료: box_01 안전거리 1.29m 접근 완료 |
| bump | PASS | 체험 자동실행 완료: box_02 3회 충돌·후퇴 완료, 객체 이동=1.387m |
| mass | PASS | 체험 자동실행 완료: 질량 비교 완료 · 10kg 이동 0.422m / 500kg 이동 0.145m |
| mobility | PASS | 체험 자동실행 완료: 물리 속성 비교 완료 · Movable 이동 0.610m / Fixed 이동 0.000m |
| appearance | PASS | 체험 자동실행 완료: box_04 크기 × 1.25 |
| temperature | PASS | 체험 자동실행 완료: 습도 미션 COMPLETED, Push 2/2 |
| worker_safety | PASS | 체험 자동실행 완료: Worker 안전 상태 기록: CLEAR, EMERGENCY_ESCAPE, EMERGENCY_STOP, SLOW_TURN · 우회 통과 완료를 의미하지는 않습니다. |

각 체험 후 다음 체험 준비도 모두 통과했습니다. 원본 기록: `8_experiences_final_verified.jsonl`.

## 실제 한국어 명령

| 명령 | 이동량 | 회전각 | 결과 |
|---|---:|---:|---|
| 뒤로 1m 후진해 | -0.900m | 0.0° | PASS |
| 앞으로 1m 가 | 0.900m | 0.0° | PASS |
| 왼쪽으로 45도 돌아 | 0.000m | 42.2° | PASS |
| 5미터 앞으로 주행해 | 4.900m | 0.0° | PASS |

현재 명령 도착 허용오차는 거리 0.10m, 회전 3°입니다. 결과는 실제 측정값입니다.

## Nav2 실제 전환·주행

- Nav2 ready=true, 실제 모드 전환 후 [-4.000000476837158, -3.0000016689300537]에서 [-2.0, -3.0] 목표로 주행.
- 종료 위치 [-2.2496001720428467, -2.992237091064453], 목표 오차 0.250m, 약 17.2초, 정상 정지.
- 목표점 클릭 주행의 도착 허용범위 0.25m 안에서 실제 위치로 판정. 원본 기록: `nav2_live_goal.json`.
- 충돌 입력은 USD 장면 경계에서 만든 시뮬레이션 ground-truth pointcloud이며 실제 LiDAR 측정은 아닙니다.

## 회귀·패키지 검증

- Python: 주행 모드 10, 경계/우회 5, Worker 안전 9, Nav2 자동 시작 1, 웹 35개 통과.
- Isaac USD Python: 카메라·바닥 3, 실제 Worker 0.5/1/2배 발 높이·반복 보정 1개 통과.
- 홈페이지: 최종 production 빌드 및 Node 테스트 57개 통과.
- 세 브라우저 화면 JavaScript 구문 검사 통과.
- ZIP 빌더는 CRC와 SHA256을 검사합니다. 두 파일은 release/LATEST_RELEASE_PAIR.json으로 묶습니다.

## 승인 후 예정 작업

- 기존 Spark ZIP 파일명을 유지해 GitHub 릴리스 자산·체크섬·업데이트 메타데이터를 갱신.
- GitHub 동반 홈페이지 파일과 Spark 릴리스 노트 갱신.
- 홈페이지 로컬 변경을 커밋·푸시한 정확한 소스로 Sites 버전을 저장하고 게시.
- 이 PC release 폴더에 브라우저/Spark ZIP과 홈페이지 전체 소스 ZIP을 함께 보관. 이전 아카이브는 삭제하지 않음.

## 설정 근거

- [Nav2 Jazzy DWB 문서](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/)
- [Nav2 Jazzy 충돌 감시기 문서](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)
