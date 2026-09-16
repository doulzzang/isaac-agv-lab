# Isaac Sim을 이용한 물류창고 AGV Robot Agentic 체험

## 2026-09-16 Ubuntu 22.04 / Isaac 4.5 · x86_64

[다운로드 ZIP](https://github.com/doulzzang/isaac-agv-lab/releases/download/v2026.09.16-isaac45/isaac_interactive_physics_lab_ubuntu22_isaac45_20260916.zip) · [릴리즈 설명](https://github.com/doulzzang/isaac-agv-lab/releases/tag/v2026.09.16-isaac45)

현재 Ubuntu 22.04.5 / Isaac 4.5에서 실제 시험한 최신 별도 배포본입니다. 자동체험 비교 초기화 오류 수정, Worker 관찰 30초 및 실시간 진행 표시를 포함합니다. Worker 회피 동작·주행 재개까지 확인했으며 원래 경로 복귀는 미검증입니다. 대학원동·개인 저장 장면은 포함하지 않습니다. 이후 `room_assets/`에 추가할 수 있습니다. 설치·사용법과 검증 범위는 ZIP의 README.md / VALIDATION.md를 확인하세요. 기존 Spark 배포본은 변경하지 않습니다.

아래 2026-09-09 안내는 과거 배포본 기준입니다. 최신 4.5 설치에는 위 링크와 ZIP 내부 설명을 사용하세요.

2026-09-14: 모든 배포 ZIP의 조작 화면 제목을 정식 명칭으로 갱신했습니다. 파일명·다운로드 링크는 유지하며 SHA256은 변경되었습니다.

## 2026-09-15 Spark 5.1 / 6.0 갱신

기존 파일명과 다운로드 링크를 유지하고 ZIP 내용과 SHA256만 교체했습니다.

- local / homepage: 객체·공간 업로드와 목록 분리, 항상 보이는 객체 드롭 영역
- 전방·좌우 명령 해석, 거절·실패 이유를 메시지 이력에 표시
- 습도 버튼 옆 연습·USB 값과 변화량 표시
- 로봇 시작 위치 복원, 확인 후 현재 공간 초기화
- 공간 편집 ‘저장 · 다음 실행에 복원’ / ‘저장 안 함’

검증: 테스트 45개, 실제 USD 저장/복원, 모바일·데스크톱 격리 브라우저, 홈페이지 빌드. 실제 DGX Spark GPU 주행은 미검증입니다.

홈페이지 서버 수정 파일은 [homepage_update](homepage_update/README.md)에 있습니다. ZIP 설치만으로 운영 중인 홈페이지 서버가 갱신되지는 않으며, 홈페이지 프로젝트에서 반영·게시해야 합니다. 개인 저장 장면은 `saved_scenes/`, 연결 및 시작 장면 설정은 `.web_runtime/`에 있으므로 기존 설치 갱신 시 보관하세요.

## DGX Spark ARM64 배포본 (2026-09-13)

실제 Spark GPU 실행은 미검증인 API 이전/설치 수정본입니다. 대학원동 자료 미포함, 이후 USD 추가 가능.

- [Spark · Isaac 5.1 ZIP](https://github.com/doulzzang/isaac-agv-lab/releases/download/v2026.09.13-spark/isaac_interactive_physics_lab_spark_isaac51_20260913_154402.zip)
- [Spark · Isaac 6.0 ZIP](https://github.com/doulzzang/isaac-agv-lab/releases/download/v2026.09.13-spark/isaac_interactive_physics_lab_spark_isaac60_20260913_154403.zip)
- [Spark 릴리즈 설명](https://github.com/doulzzang/isaac-agv-lab/releases/tag/v2026.09.13-spark)

Spark ZIP은 ARM64 Python 3.12 / CUDA 13 PyTorch / ROS 2 Jazzy용입니다. 아래의 2026-09-09 ZIP은 x86_64용이므로 Spark에서는 사용하지 마세요.


Isaac Sim AGV 체험 프로그램의 배포 저장소입니다. 실행 소스와 AI대학원동 USD 자산은 아래 릴리즈 ZIP에 함께 들어 있습니다. 이 저장소를 복제하는 것만으로는 실행할 수 없으며, 대상 환경에 맞는 ZIP을 내려받아 압축을 풀어 설치하세요.

- [최신 릴리즈 및 ZIP 다운로드](https://github.com/doulzzang/isaac-agv-lab/releases/latest)
- [Ubuntu 22.04 / Isaac 4.5 ZIP](https://github.com/doulzzang/isaac-agv-lab/releases/download/v2026.09.09/isaac_interactive_physics_lab_ubuntu22_isaac45_20260909_100106.zip)
- [Ubuntu 24.04 / Isaac 5.1 ZIP](https://github.com/doulzzang/isaac-agv-lab/releases/download/v2026.09.09/isaac_interactive_physics_lab_ubuntu24_isaac51_20260909_100115.zip)

각 ZIP 약 676MiB. 소스 공개 배포본이며, 인증키·개인별 연결 설정은 포함하지 않습니다.
5.1용은 이 4.5 운영 PC에서 실제 시뮬레이션 실행을 검증하지 않았습니다.

---

# AGV 체험 프로그램 — 다른 PC 설치·빌드·첫 사용

이 ZIP은 소스를 포함한 배포본입니다. 별도 컴파일 없이 설치 후 실행합니다.
대상은 INSTALL_TARGET.json을 확인하세요. 22.04용은 Isaac 4.5, 24.04용은 Isaac 5.1입니다.
Isaac/NVIDIA 드라이버는 별도 설치해야 합니다. 5.1용 실제 시뮬레이션은 이 4.5 PC에서 검증할 수 없습니다.

## 1. 폴더 배치와 설치

아래 명령은 ZIP을 푼 isaac_interactive_physics_lab 폴더 안에서 실행합니다.
예를 들어 두 폴더를 나란히 배치하면 사용자명이나 바탕화면 절대 경로가 필요 없습니다.

```text
작업폴더/
  isaac-sim-4.5/                 # 24.04용은 isaac-sim-5.1/
  isaac_interactive_physics_lab/
    README.md
    install.sh
    requirement.txt
    requirements.txt
    room_assets/
    exts/
    web_app/
    tools/
```

```bash
cd isaac_interactive_physics_lab
bash install.sh --isaac-root ../isaac-sim-4.5
# Ubuntu 24.04 / Isaac 5.1 ZIP에서는 대신 다음 실행:
# bash install.sh --isaac-root ../isaac-sim-5.1
```

설치 도구는 Ubuntu 패키지, 독립 NoMaD Python 환경·패키지·소스·가중치,
Nav2 환경, Ollama 및 llama3.1 모델을 설치합니다. 인터넷과 최소 25GiB 여유 공간이
필요하며 Isaac 및 ZIP 해제 공간은 별도입니다. sudo 암호를 요청할 수 있습니다.
USB 센서를 쓰면 설치 명령에 --usb-permission을 추가하고 로그아웃/로그인하세요.
설정은 .install/에 저장됩니다. 다른 PC로 옮긴 뒤 설치 명령을 다시 실행하세요.

`requirement.txt`는 `requirements.txt`를 읽는 호환 파일입니다. 시스템 Python이나
Isaac Python에 직접 pip install하지 말고 위 설치 도구를 사용하세요.
웹 서버와 홈페이지 에이전트는 Python 3.10 이상 표준 라이브러리만 사용합니다.
Nav2는 pip가 아닌 environment-nav2.yml로 설치합니다.
얼굴 생성 기능은 선택 사항으로 install_face2d.sh / install_face3d.sh를 별도 사용합니다.

점검만 하기: `bash install.sh --check --isaac-root ../isaac-sim-4.5`
일부 기능 설치 생략: --skip-ollama / --skip-nav2 / --skip-learned (해당 기능은 사용할 수 없음).

## 2. 실행과 첫 장면

```bash
bash run_isaac_with_browser.sh --local
# 홈페이지 이용 시:
# bash run_isaac_with_browser.sh --homepage
```

옵션을 생략하면 홈페이지 연결 설정이 있을 때 홈페이지, 없으면 로컬을 엽니다.
처음에는 장면 생성의 **Lab 1-B Warehouse**를 누르고 완료 메시지를 기다리세요.
가벼운 시험은 **Lab 1-A Minimal**입니다. 그다음 **Lab 1-C Add AGV**로 로봇과
카메라를 생성합니다. 장면 생성/교체 중에는 다른 생성 버튼을 누르지 마세요.

## 3. 장면 교체

방·전체 공간 교체 → 공간 종류에서 기존 창고/교실/AI대학원동 선택 →
**이 공간으로 교체** → 완료 후 **Lab 1-C Add AGV**.
AI대학원동 USD는 room_assets/에 포함되어 있으므로 파일을 다시 첨부할 필요가 없습니다.
AI대학원동은 주행 전용이며 Worker·습도 미션·자동체험을 실행하지 않습니다.
교실과 창고에서는 기존 체험을 사용할 수 있습니다.

새 USD/USDA/USDC/USDZ/ZIP은 공간 추가 영역에 드래그합니다(웹 업로드 최대 20MB).
큰 파일은 운영 PC의 room_assets/에 복사한 뒤 목록을 새로고침하세요.
ZIP은 운영 PC에서 풀며 지원 USD 파일을 목록에 등록합니다. 참조 텍스처 등은
상대 폴더 구조를 유지해 함께 넣어야 합니다. 목록 등록만으로 장면이 바뀌지는 않습니다.
현재 장면 저장은 별도로 사용하세요. 이번 릴리즈는 마지막 선택 장면 자동 복원을 보장하지 않습니다.


### AI대학원동 파일 경로

모든 경로는 ZIP 안의 `isaac_interactive_physics_lab/` 폴더를 기준으로 합니다.

- 건물 USD: `./room_assets/ForDemo_AI_Graduate_Building.usd`
- 목록 이름 설정: `./room_assets/ForDemo_AI_Graduate_Building.usd.json` (`AI대학원동`)
- 실행 파일: `./run_isaac_with_browser.sh`
- 설치 파일: `./install.sh`
- 로컬 조작 화면: `./exts/edu.interactive.physics/assets/web_control/index.html`
- 독립 웹 조작 화면: `./web_app/index.html`

```text
isaac_interactive_physics_lab/
├── README.md
├── install.sh
├── run_isaac_with_browser.sh
└── room_assets/
    ├── ForDemo_AI_Graduate_Building.usd
    └── ForDemo_AI_Graduate_Building.usd.json
```

홈페이지나 브라우저에서는 경로를 직접 입력하지 않고 **공간 종류 → AI대학원동 →
이 공간으로 교체**를 누르면 됩니다. 운영 PC가 자신의 설치 폴더에서 파일을 찾습니다.
포함 파일은 외부 참조와 작은 소품을 정리한 현재 사용본이며, 원본 백업은 포함하지 않습니다.

## 4. AGV 조작

화면 ↑/↓는 전진·후진, ←/→는 회전입니다. 누르고 있으면 계속 입력하며 놓으면 해제합니다.
짧은 클릭도 짧은 이동 명령으로 전달합니다. 충돌이나 지연으로 실제 이동 거리는 달라집니다.
키보드 운전 활성화 후 방향키를 사용할 수 있고 STOP 또는 Space로 정지합니다.
LLM 입력란에서 방향키를 누르면 글 편집에 사용됩니다.
자연어 예: '앞으로 1미터 이동해'. 장애물에 막혔는지 카메라와 메시지 이력을 확인하세요.
Reset AGV는 생성 당시 위치로 돌아갑니다.

객체·Worker 편집 영역에 모델 파일을 드래그한 뒤 새 객체 추가 또는 선택 객체에 적용을 누릅니다.
Isaac에서 선택한 USD 객체는 표시된 대상 경로를 확인하고 선택 객체 삭제로 장면에서 제외합니다.
원본 USD 파일 자체는 바꾸지 않습니다. 로컬의 창고 화면 → 패널 복원으로 Isaac 편집 패널을 복원합니다.

## 5. 홈페이지 및 다른 PC

홈페이지 방문 PC는 Isaac 설치 없이 웹브라우저로 AGV 메뉴에 접속하면 됩니다.
운영 PC의 --homepage 최초 실행 시 터미널의 12자리 코드를 홈페이지 소유자 관리자 계정으로
AGV → 운영 PC 연결 설정에 입력합니다. 등록 후 인증은 자동입니다.
연결 설정 파일과 인증키는 ZIP에 포함하지 않습니다. 각 운영 PC를 직접 등록하세요.
첫 방문자는 자동 입장, 동시 추가 방문자는 홈페이지 관리자 승인 대상입니다.
승인 기록이 남은 브라우저는 재승인을 생략할 수 있습니다. 조작 요청으로 한 명만 운전합니다.
LAN 웹 서버 사용은 web_app/README.md를 참고하세요. 홈페이지 전체 소스는 이 Isaac ZIP과 별개입니다.

## 6. 다른 PC에서 ZIP 다시 빌드

프로젝트 루트에서 시스템 python3으로 실행합니다. 추가 pip 패키지는 필요 없습니다.

```bash
python3 tools/build_release.py --target ubuntu22_isaac45
python3 tools/build_release.py --target ubuntu24_isaac51
```

결과는 ./release/의 ZIP과 SHA256 파일입니다. room_assets/를 포함하며 로컬 인증·가상환경·
캐시는 제외됩니다. USD 텍스트 파일의 프로젝트 내부 절대 참조는 상대 참조로 변환합니다.
외부 폴더 참조는 누락된 자료로 빌드를 중단하므로 프로젝트 자산 폴더에 넣어 수정하세요.
ZIP CRC와 파일별 SHA256은 빌드 도구가 검증합니다.
