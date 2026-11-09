# WSL ROS2 조회·통신 설정 기록

적용 대상은 Windows의 Ubuntu22.04/ROS2 Humble/Fast DDS 학습 환경이다.
Jetson에 이 WSL 전용 설정을 그대로 적용해야 한다는 뜻은 아니다.
현재 PC의 도메인은73이며 아래 포트 허용 범위도 도메인73의 기본 DDS 포트를 기준으로 한다.

## 문제와 검증 범위

- 데몬이 없는 상태의 localhost TCP 접속이 즉시 거절되지 않아 기본 CLI 조회가 오래 대기했다.
- 데몬을 직접 띄우는 것과 단순히 발견 시간을 늘리는 것만으로 재발을 해결하지 못했다.
- Hyper-V UDP 허용 뒤에도 기본 전송/UDP 전용에서 간헐적 발견 실패가 남았다.
- 최대 UDP 메시지를1400바이트로 제한한 대조 시험은2회 성공했고 기본 크기 시험은2회 실패했다.
- 실제 셧다운·재시작2회에서 새 터미널의 node/topic 조회 및 turtlesim pose 수신을 확인했다.
- 향후 OS·네트워크·DDS 버전 변경까지 정상이라고 보장하는 기록은 아니다.

관련 공식 자료: [ROS CLI 미러 네트워크 이슈](https://github.com/ros2/ros2cli/issues/934),
[Microsoft WSL 네트워크](https://learn.microsoft.com/en-us/windows/wsl/networking),
[Fast DDS 2.6 전송 XML](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html).

## 파일

- `fastdds_udp_only.xml`: 기본 전송 대신 UDPv4 사용, `maxMessageSize=1400`.
- `cli_daemon_autostart.bash`: 프로필 환경 변수 설정 및 ros2 셸 함수.
  node/topic list는 기본적으로3초 직접 발견을 사용한다. 명시한 spin-time은 유지한다.
  다른 graph 명령은 데몬 사전 시작으로 TCP 대기 지점을 피한다.

## 적용 방법

WSL에서 저장소 루트를 현재 경로로 두고 실행한다.

```bash
mkdir -p ~/.config/ros
cp learning/wsl/fastdds_udp_only.xml ~/.config/ros/
cp learning/wsl/cli_daemon_autostart.bash ~/.config/ros/
```

`~/.bashrc`에 다음이 이미 있으면 중복 추가하지 않는다.

```bash
export ROS_DOMAIN_ID=73
if [ -f "$HOME/.config/ros/cli_daemon_autostart.bash" ]; then
    . "$HOME/.config/ros/cli_daemon_autostart.bash"
fi
```

현재 열려 있는 각 터미널에서 `source ~/.bashrc`를 실행한다. 새 터미널에는 자동 적용된다.
이 셸 함수는 대화형 Bash에 적용된다. 별도 서비스·비대화형 셸에서는 전송 프로필 환경을 명시적으로 전달해야 한다.
ROS 설치 파일, 펌웨어, ROS_DOMAIN_ID 자체를 다른 값으로 바꾸는 설정은 아니다.

Windows 사용자 `.wslconfig`의 기존 다른 옵션을 보존하면서 미러 모드를 사용하는 구성이다.

```ini
[wsl2]
networkingMode=mirrored
```

관리자 PowerShell에서 도메인73의 UDP 포트 규칙을 한 번 추가한다. 같은 이름이 이미 있으면
먼저 `Get-NetFirewallHyperVRule -Name "ROS2-Domain73-UDP"`로 확인하며 중복 생성하지 않는다.

```powershell
New-NetFirewallHyperVRule `
  -Name "ROS2-Domain73-UDP" `
  -DisplayName "ROS2 Domain 73 UDP" `
  -Direction Inbound `
  -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
  -Protocol UDP `
  -LocalPorts '25650-25899' `
  -Action Allow
```

방화벽 전체를 해제하는 규칙이 아니다. 다른 도메인이나 비기본 DDS 포트 설정은 별도 확인한다.

## 재시작 후 확인

`wsl --shutdown`은 실행 중인 모든 WSL 노드를 종료한다. 재시작 뒤 첫 터미널에서:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

두 번째 터미널에서:

```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /turtle1/pose --once
```

각 터미널의 `echo "$ROS_DOMAIN_ID"`가73인지 확인한다. 시험이 끝나면 turtlesim 터미널에서 Ctrl+C로 종료한다.
ROS CLI는 실제 데이터와 별개의 조회 데몬도 사용하므로 빈 목록·대기·메시지 미수신을 구분한다.

## 되돌리기

`.bashrc`의 도우미 source 블록을 제거하고 새 셸을 연다. 이미 연 셸에서는 필요시
`unset -f ros2` 및 `unset FASTRTPS_DEFAULT_PROFILES_FILE`로 해당 셸의 설정을 해제한다.
방화벽 규칙까지 취소하려면 관리자 PowerShell에서 이 작업이 만든 이름만 제거한다.

```powershell
Remove-NetFirewallHyperVRule -Name "ROS2-Domain73-UDP"
```
