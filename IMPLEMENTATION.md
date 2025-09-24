# ESP32 드론 통합 시스템 구현 문서

## 📋 프로젝트 개요

기존 ESP32 Crazyflie 드론 시스템을 Unity 드론 시뮬레이션과 연동할 수 있도록 개조한 프로젝트입니다.
Unity → GCS Server → ESP32 드론의 3단계 중계 구조를 통해 Unity 시뮬레이션의 드론 0번 명령이 실제 물리적 ESP32 드론에 전달됩니다.

## 🏗️ 시스템 아키텍처

```
┌─────────────────┐    WebSocket     ┌─────────────────┐    TCP Client    ┌─────────────────┐
│   Unity Web     │   Port 5089      │   GCS Server    │    Port 8080     │   ESP32 Drone   │
│   (WDSS-web)    │ ───────────────► │   (C# .NET)     │ ◄─────────────── │   (ESP-IDF)     │
│                 │                  │                 │                  │                 │
│ • 드론 시뮬레이션   │                  │ • WebSocket     │                  │ • LED 제어      │
│ • JSON 생성     │                  │   서버 (5089)   │                  │ • 모터 제어     │
│ • UI 컨트롤     │                  │ • TCP 서버      │                  │ • WiFi 연결     │
│                 │                  │   (8080)        │                  │ • JSON 파싱     │
└─────────────────┘                  └─────────────────┘                  └─────────────────┘

AWS EC2 Instance                      AWS EC2 Instance                    Physical Hardware
Frontend + Backend                    GCS Server                          ESP32-S2 Board
```

## 🔧 주요 구현 사항

### 1. ESP32 드론 펌웨어 (ESP-IDF)

#### 1.1 아키텍처 변경
- **기존**: Crazyflie 복잡한 비행 제어 시스템
- **변경**: 최소한의 LED + 모터 제어 시스템
- **이유**: 메모리 부족 (9000 bytes DRAM overflow) 해결

#### 1.2 핵심 컴포넌트
```c
// main/main.c - 통합 애플리케이션 (단일 파일 구조)
- WiFi Station 모드 초기화
- TCP 클라이언트 연결
- JSON 명령 파싱 및 실행
- LED RGB 제어 (Red/Green/Blue, 항상 표시)
- 모터 속도 제어 (안전을 위해 최대 8% 제한)
- 연결 모니터링 및 자동 재연결

// config.h - 환경 설정 (Git 제외)
- WiFi SSID/Password
- 서버 IP 주소
- 환경별 설정 분리

// CMakeLists.txt - 의존성 관리
- driver, led, motors, esp_wifi, nvs_flash, json
- 컴파일 에러 방지를 위한 최적화된 의존성 구조
```

#### 1.3 네트워크 설정
```c
// WiFi 설정
#define WIFI_SSID "현장_WiFi_이름"      // config.h에서 설정
#define WIFI_PASS "현장_WiFi_비밀번호"   // config.h에서 설정

// TCP 서버 설정
#define TCP_SERVER_IP "3.36.114.187"    // AWS GCS 서버
#define TCP_SERVER_PORT 8080            // TCP 포트
```

#### 1.4 JSON 프로토콜
```json
{
  "motor_speed": 30,           // 0-100% (실제로는 8% 제한)
  "rgb": {
    "r": 255,                  // 0-255 Red
    "g": 0,                    // 0-255 Green
    "b": 0                     // 0-255 Blue
  }
}
```

#### 1.5 LED 제어 로직
```c
// RGB 값에 따라 단일 색상 LED 제어 (항상 표시)
if (red == 0 && green == 0 && blue == 0) {
    // 모든 값이 0이면 기본으로 RED 표시
    ledSet(LED_RED, true);
} else if (red >= green && red >= blue) {
    ledSet(LED_RED, true);      // 빨간색 LED
} else if (green >= blue) {
    ledSet(LED_GREEN, true);    // 초록색 LED
} else {
    ledSet(LED_BLUE, true);     // 파란색 LED
}
// 주요 개선: 128 임계값 제거로 LED OFF 상황 완전 제거
```

### 2. GCS 서버 (C# ASP.NET Core)

#### 2.1 새로 추가된 서비스
```csharp
// TcpServerService.cs - ESP32 TCP 서버
public class TcpServerService : BackgroundService
{
    private readonly int tcpPort = 8080;        // ESP32 연결용 포트
    private TcpListener tcpListener;
    private TcpClient esp32Client;
    private NetworkStream esp32Stream;
}
```

#### 2.2 WebSocket 미들웨어 수정
```csharp
// WebSocketMiddleware.cs 주요 변경사항
- ProcessJsonForESP32() 함수 추가
- Unity JSON → ESP32 TCP 전달 로직
- Null 참조 에러 방지 (gcsRoot?.jsonSystem != null)
- MemoryStream 타입 변환 처리
```

#### 2.3 포트 구성
- **5089 포트**: WebSocket (Unity ↔ GCS)
- **8080 포트**: TCP Server (GCS ↔ ESP32)

### 3. WDSS-web 프론트엔드

#### 3.1 백엔드 라우터 수정
```python
# backend/app/routers/project.py
async def send_json_to_external_server(json_data: str):
    # Unity → GCS 전송
    uri = "ws://3.36.114.187:5089/json"    # AWS 배포용
    # uri = "ws://localhost:5089/json"     # 로컬 테스트용
```

## 🔐 보안 및 설정 관리

### 1. 민감한 정보 분리
```bash
# ESP32 프로젝트
main/config.h          # Git 제외 - 실제 WiFi 정보
main/config.example.h  # Git 포함 - 템플릿
.gitignore            # config.h 제외 설정
```

### 2. 환경별 설정
```c
// config.example.h (템플릿)
#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define SERVER_IP "192.168.1.100"        // 로컬용
// #define SERVER_IP "AWS_SERVER_IP"     // 배포용

// config.h (실제 사용)
#define WIFI_SSID "Thin2.4G"
#define WIFI_PASS "qweasdzxc"
#define SERVER_IP "3.36.114.187"         // AWS 서버
```

## 🚀 배포 및 사용법

### 1. 개발 환경 설정
```bash
# ESP32 프로젝트 설정
cd esp-drone-wdss
cp main/config.example.h main/config.h
# config.h 편집하여 WiFi 정보 입력

# 빌드 및 플래시
idf.py build
idf.py flash
idf.py monitor
```

### 2. AWS 배포 설정
- **GCS 서버**: 3.36.114.187 (포트 5089, 8080 오픈)
- **WDSS-web**: 프론트엔드/백엔드 배포
- **ESP32**: config.h에서 AWS IP 설정

### 3. 네트워크 요구사항
- **ESP32**: 2.4GHz WiFi 필수 (ESP32-S2 제한사항)
- **방화벽**: 8080(TCP), 5089(WebSocket) 포트 오픈

## 🛠️ 주요 해결 과제

### 1. 기술적 문제
- **메모리 부족**: Crazyflie 시스템 제거, 최소한 컴포넌트만 사용
- **WiFi 호환성**: 5GHz → 2.4GHz WiFi 환경 확인
- **네트워크 통신**: WebSocket → TCP 변경 (NAT 문제 해결)
- **컴파일 에러**: stringop-overflow 경고 처리
- **아키텍처 단순화**: gcs_comm 컴포넌트 완전 제거, main.c 단일 파일 구조로 변경
- **의존성 최적화**: CMakeLists.txt driver 의존성 추가로 빌드 에러 해결
- **LED 제어 개선**: 항상 LED 표시되도록 로직 개선 (OFF 상황 제거)

### 2. 보안 문제
- **민감한 정보**: WiFi 정보를 config.h로 분리
- **Git 관리**: .gitignore 설정으로 보안 정보 제외

### 3. 시스템 통합
- **포트 통일**: Unity(5089) → GCS(5089,8080) → ESP32(8080)
- **JSON 프로토콜**: 일관된 데이터 형식 정의
- **에러 처리**: Null 참조, 연결 실패 등 예외 상황 처리

## 📊 성능 및 안전성

### 1. 모터 안전성
```c
// 안전을 위한 모터 출력 제한
if (speed_percent > 8) {
    speed_percent = 8;    // 최대 8%로 제한
}
```

### 2. 연결 안정성
```c
// WiFi 자동 재연결
static void wifi_event_handler() {
    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();    // 자동 재연결
    }
}

// TCP 연결 모니터링
if (activity == 0) {
    const char* ping = "ping\n";
    send(tcp_socket, ping, strlen(ping), 0);    // 연결 상태 확인
}
```

## 🎯 시스템 동작 흐름

### 1. 초기화 순서
1. **ESP32**: WiFi 연결 → TCP 서버 접속 → 대기 모드
2. **GCS**: WebSocket 서버(5089) + TCP 서버(8080) 시작
3. **Unity**: 웹 브라우저에서 접속 → WebSocket 연결

### 2. 명령 전달 과정
1. **Unity**: 드론 시뮬레이션 실행 → JSON 생성
2. **WDSS-web**: JSON → GCS WebSocket(5089) 전송
3. **GCS**: JSON 수신 → ESP32 TCP(8080) 전달
4. **ESP32**: JSON 파싱 → LED/모터 제어 실행

### 3. 실시간 동기화
- Unity 드론 0번의 색상 변경 → ESP32 LED 색상 변경
- Unity 드론 0번의 속도 변경 → ESP32 모터 속도 변경
- 지연시간: < 100ms (로컬 네트워크 기준)

## 📈 확장 가능성

### 1. 다중 드론 지원
- 현재: 드론 0번만 지원
- 확장: 드론 ID 필드 추가하여 다중 드론 제어

### 2. 센서 데이터 피드백
- 현재: 단방향 제어 (Unity → ESP32)
- 확장: 양방향 통신 (ESP32 센서 → Unity)

### 3. 고급 제어 기능
- 현재: RGB LED + 모터 속도
- 확장: 위치 제어, 자세 제어, 경로 추적

---

## 📝 결론

이 프로젝트는 Unity 시뮬레이션과 실제 하드웨어 간의 실시간 연동을 성공적으로 구현했습니다.
특히 네트워크 통신의 복잡성을 해결하고, 보안 및 배포 관련 모범 사례를 적용하여
실제 운영 환경에서 사용 가능한 안정적인 시스템을 완성했습니다.

**핵심 성과:**
- ✅ ESP32 Crazyflie 시스템 → 단순 LED/모터 제어 시스템 개조
- ✅ GCS Server에 ESP32 TCP 중계 서버 추가
- ✅ Unity JSON 명령 → ESP32 하드웨어 제어 3단계 중계 구조 구축
- ✅ WiFi 설정 분리 및 Git 보안 관리 (config.h 시스템)
- ✅ WebSocket → TCP 프로토콜 변경으로 NAT 문제 해결
- ✅ 메모리 최적화 (9000 bytes 오버플로우 → 189KB 바이너리)
- ✅ 아키텍처 단순화: gcs_comm 컴포넌트 제거, main.c 단일 파일 구조
- ✅ LED 제어 로직 개선: 항상 LED 표시 보장 (OFF 상황 완전 제거)
- ✅ 빌드 시스템 최적화: CMakeLists.txt 의존성 구조 개선

이 시스템은 향후 대규모 드론쇼나 교육용 드론 플랫폼으로 확장할 수 있는
견고한 기반을 제공합니다.