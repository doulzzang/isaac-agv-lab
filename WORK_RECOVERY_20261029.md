# 2026-10-29 작업 재개 기록

프로젝트: /home/gistai01/Desktop/isaac_interactive_physics_lab

## 다음 세션에서 먼저 할 일
이 파일과 AGENTS.md, validation_artifacts/20261029/VALIDATION_REPORT.md를 읽고 이어서 작업한다. 이전 릴리스 노트를 다시 요청하지 않는다.

## 지금 사용자가 재로그인하려는 이유
USB 습도 센서 Permission denied 문제. /etc/group의 dialout:x:20:gistai01 등록을 확인했다. 사용자가 sudo usermod -aG dialout gistai01 및 sudo setfacl -m u:gistai01:rw /dev/ttyUSB0 처리했다고 알렸다. 프로그램만 재실행한 후 값이 안 들어온다고 함. 새 그룹 권한을 로그인 세션에 적용하기 위해 Ubuntu 로그아웃/로그인 또는 재부팅 안내. 다음 단계는 재로그인 후 센서 수신 확인. 실제 수신 성공은 아직 확인하지 않음.

## 센서 정보
- CH340 USB: vendor 1a86, product 7523, ttyUSB0.
- /run/udev/data/c188:0에서 serial/by-id/usb-1a86_USB_Serial-if00-port0 등록 확인.
- 코드 기본 포트와 일치. Modbus 9600 baud, slave 1, holding registers 0부터 2개 읽음.
- 코드: exts/edu.interactive.physics/edu_interactive_physics/modbus_temperature.py
- 현재 센서 리더는 실패 시 자동 재시도함.
- Codex sandbox에서는 /dev/ttyUSB0이 숨겨지고 TCP/UDP socket 생성도 EPERM. 사용자 프로그램의 실제 수신 실패와 혼동하지 말 것.

## 완료한 작업
- Codex bwrap 오류: 사용자가 sudo apparmor_parser -r /usr/share/apparmor/extra-profiles/bwrap-userns-restrict 실행 후 shell 복구.
- codex-cli 0.165.0에서 테스트 명령은 codex sandbox -- /usr/bin/true. codex sandbox linux 형태는 linux를 실행 파일로 오인함.
- 후진 회귀 실패 재현 후 extension.py의 _drive_robot_to_raw 수정: speed<0일 때 목표 방향에 pi 추가, abs(speed) 기준 감속 후 부호 복원.
- 주행 10, 경계/우회 5, Worker 9, Nav2 자동 시작 1 테스트 통과.
- 사용자가 일반 터미널에서 실행한 web_tests.log 직접 확인: 웹 35개 전체 통과. 합계 60개.
- 센서 테스트 test_modbus_temperature.py 18개 추가 통과.
- 이번 수정에 대한 Isaac Sim 실제 후진 주행 검증은 아직 하지 않음.

## 실행 로그
/home/gistai01/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1/kit_20261029_151559.log 최신 확인.
이전 kit_20261029_143958.log에서 현재 Desktop 프로젝트 확장 로드, 제어 서버 8765 초기화, 저장 장면/AGV 복원 확인.
Worker 재질 OmniRL_PBR.mdl이 예전 /home/gistai01/isaac_interactive_physics_lab/isaac_interactive_physics_lab 경로를 참조하여 로드 오류 있음. 아직 수정하지 않음.

## 릴리스 실제 상태
9월17일 릴리스 노트에는 게시 대기라고 쓰였으나 release/PUBLICATION_RECEIPT_20260917.json 및 LATEST_RELEASE_PAIR.json에는 게시 완료 기록 있음. 로컬 기록상 홈페이지 버전146 및 GitHub 게시 완료. 현재 원격 상태는 검증하지 않음.
기존 Spark ZIP 및 홈페이지 ZIP 모두 SHA256/전체 CRC 통과. 기존 아카이브 변경 없음.
현재 작업 폴더 .git은 sandbox 내에서 빈 보호 디렉터리로 보여 git status 불가. /tmp/doulzzang-homepage-20260916 없음. Git 메타데이터가 삭제됐다고 단정하지 말 것.
이번 로컬 수정은 기존 ZIP에 미포함. 새 ZIP 빌드/커밋/푸시/게시 미실행.

## 사용자 지시
커밋/푸시/릴리스 교체/홈페이지 게시 전 검증된 deliverables를 제시하고 명시 승인 필요. continue는 게시 승인 아님.
게시 전에 tools/build_release_pair.py로 Spark/browser 및 홈페이지 소스 ZIP 두 개 준비. 기존 Spark 이름 및 이전 아카이브 보존.
제목과 Worker 20초 조건은 AGENTS.md 준수.
사용자는 반복적인 수동 명령 요청에 매우 불편함을 표현. 직접 읽을 수 있는 파일/로그는 먼저 직접 확인하고, 외부 실행이 꼭 필요하면 한 번에 묶어 요청.

## 센서 후속 확인 및 표시 수정
- 사용자 재부팅 후 일반 터미널 id에서 dialout 적용, /dev/ttyUSB0 root:dialout rw 확인.
- 사용자가 /status 실제 조회 결과 제공: hardware_sensor_error 빈 문자열, humidity 43.7, sample_count 245, sample_age .8337. 실제 센서 통신 정상.
- 사용자는 문제를 화면 표시라고 명확히 함.
- 세 HTML에서 USB 온습도 표시를 .temperature 미션 section에서 분리, 독립 sensor-readout section에 배치. 대학원동에서 미션 숨길 때 실측 센서까지 숨기는 구조 수정.
- 구조 검사 및 실제 JS 렌더링 함수 GJS 실행 검증 통과(세 HTML), 센서 테스트18 및 대학원동 테스트2 통과.
- 브라우저 새로고침 후 실제 화면 확인 필요. Isaac 재시작 불필요: 로컬 HTML은 요청마다 파일에서 읽음.

- 사용자 최종 확인: 습도 화면 표시 정상 동작. 센서 수신 및 화면 표시 문제 해결 확인.

## 저장 장면 GIST 로고 검은색 문제 수정
- 사용자 요청으로 Git/릴리스 업로드 계속 보류.
- 현재 자동 복원 장면의 GISTWallSign, GISTWallSignLeft, GISTWallSignBack, GISTFloorSign의 inputs:file이 이전 프로젝트 절대경로를 가리킴. 현재 설치 폴더의 gist_wall_logo.png 파일은 존재함.
- room_scene.restore_snapshot에서 익명 flattened 레이어의 누락된 번들 자산 경로를 현재 assets 경로로 복구. 기존 외부 자산, 존재하는 경로, 없는 대체 파일, assets 밖 경로는 보존.
- 실제 저장 장면에서 로고 4개 경로가 모두 현재 파일로 연결됨 확인. 저장 장면 파일의 SHA256 불변 확인.
- test_snapshot_asset_relocation.py: 실제 Isaac USD 라이브러리 환경에서 3개 통과(자산/배열/시계열, 경로 경계, 장면 배치 및 원본 보존, 실제 장면 로고4개).
- 실제 렌더링 화면은 Isaac Sim 재실행 후 확인 필요. 실행 중인 장면에는 아직 이 수정을 직접 적용하지 못함.
- 기존 준비된 3.5.2 ZIP에는 이 수정이 들어 있지 않아 LATEST_RELEASE_PAIR.json에 requires_rebuild=true, PUBLISH_PLAN_20261029.json에 publication_blocked 설정. 게시 도구도 보류된 plan을 거부하도록 수정. 추가 화면 확인 후 ZIP 재준비 및 최종 승인 필요.

## 사용자 제공 동신여고 이미지 벽면·바닥 매핑
- 입력: /home/gistai01/다운로드/KakaoTalk_20261029_164610331.png (282×66).
- 원본 바이트를 assets/branding/custom_wall_logo.png에 복사. SHA256 및 원본 경로는 custom_wall_logo.provenance.json에 보관.
- 사용자 추가 요청에 따라 벽3면과 바닥 모두 적용. 각 기존 로고의 중심·가로 폭·방향을 유지하고 높이를 원본 종횡비에 맞게 조정.
- 기존 저장 장면 복원 및 신규 창고 생성 모두 적용. 저장 장면 원본 파일, 객체 위치, 벽/바닥의 충돌 형상은 변경하지 않음.
- 실제 USD 환경에서 관련 테스트4개 통과(WALL_FLOOR_LOGO_TESTS.log). 실제 저장 장면의4개 자산 경로·비율과 신규 생성 경로 검증.
- 현재 실행 중인 장면에는 재불러오기/Isaac Sim 재실행이 필요하며 실제 렌더링 화면 확인은 아직 안 됨.
- Git/릴리스 게시 계속 보류. 이전 준비 ZIP은 이번 수정을 포함하지 않으며 재빌드 필요.

## 게시 준비 재개
사용자가 실행 시 업데이트 선택으로 수정 코드/장면 적용 요청. 홈페이지 게시 제외. 업데이트 테스트14개 통과. 3.5.2 새 ZIP 준비 후 tools/verify_code_release.py로 기존 게시 updater 및 새 updater 양쪽에서 실제 ZIP 설치와 저장 장면의 벽3+바닥1 이미지 연결을 확인할 예정. 최종 commit/push/release 자산 교체 전 승인 필요.

- 실제 ZIP 설치 검증: 이전 게시 updater와 새 updater 모두 PASS. 개인 설정5종 보존 및 설치 후 실제 저장 장면의 새 로고4개 경로·비율 확인. 최종 ZIP 재생성 후 해시 고정 예정. 원격 접근 git ls-remote는 github.com DNS 제한으로 실패; Git/릴리스 미게시. 일반 터미널의 publish_code_release.py --approve-code-release로 명시 승인 후 실행해야 함.
