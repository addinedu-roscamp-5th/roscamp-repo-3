# roscamp-repo-3

# RoboCallee (Robot + Callee)

스마트 매장에서 **로봇팔 + 자율 주행 로봇**을 활용해 신발 재고 운반과 고객 응대를 자동화하는 프로젝트입니다.  
고객 편의성 증대와 매장 운영 효율화를 동시에 달성하는 **로봇 기반 스마트 스토어**를 목표로 합니다.

---

## 📌 목차
1. [프로젝트 개요](#프로젝트-개요)  
2. [SW 아키텍처](#sw-아키텍처)  
3. [로봇팔 제어](#로봇팔-제어)  
4. [자율 주행 로봇 제어](#자율-주행-로봇-제어)  
5. [시연 영상](#시연-영상)  
6. [팀 소개](#팀-소개)  

---

## 1. 프로젝트 개요

- **프로젝트명**: RoboCallee  
- **프로젝트 주제**: 창고형 매장에서 운용하는 서비스 로봇  
- **목표**:
  - 로봇팔과 주행로봇을 통한 신발 박스 상하차 및 배달 자동화
  - 매장 운영 효율화 및 고객 경험 개선

![프로젝트 개요 다이어그램](./images/project_overview.png)

---

## 2. SW 아키텍처

- **구성 요소**
  - FMS (Fleet Management System) : 로봇 관제
  - Control Service : Django 웹 서비스 + PyQT GUI
  - Robot Arm Control : Hand-Eye Calibration / Pick & Place
  - Autonomous Mobile Robot : PID 제어, MAPF 기반 경로 계획

![소프트웨어 아키텍처](./images/software_architecture.png)

---

## 3. 로봇팔 제어

- **기능**
  - 선반 ↔ 버퍼 ↔ 주행 로봇 간 신발 박스 이동
  - Hand-Eye Calibration, Forward Kinematics 적용
  - EasyOCR 기반 상품 인식 및 파지(Pick & Place)

![로봇팔 제어 다이어그램](./images/robot_arm_control.png)

---

## 4. 자율 주행 로봇 제어

- **주행 전략**
  - PID 제어 기반 Navigation State Machine
  - MAPF (Multi-Agent Path Finding) 적용
  - YOLO 기반 Vision 인식 → 사람 추종 & 장애물 회피

![주행 로봇 제어 다이어그램](./images/mobile_robot_control.png)

---

## 5. 시연 영상

- [FMS 시나리오 수행 영상](http://www.youtube.com/watch?v=GBW1qsYFHe4)  
- [다중 로봇 경로 생성 영상](http://www.youtube.com/watch?v=zXWMFyJrSFo)  
- [PID 주행 제어 영상](http://www.youtube.com/watch?v=zxgvcNaVX5w)  
- [강화학습 기반 주행 영상](http://www.youtube.com/watch?v=4HWVJNYS8s4)

---

## 6. 팀 소개 (ROTUS)

- **박승우**: FMS 아키텍처 및 프로그램 개발 총괄, 비전 알고리즘 개발  
- **곽제우**: Aruco Localization, 주행 제어, MAPF  
- **이창연**: PID 제어, GUI 맵 적용  
- **문지언 / 선태욱**: 주행 시나리오 설계, YOLO 기반 Vision  
- **박민수 / 이찬종**: 로봇팔 제어, Hand-Eye Calibration, Camera Calibration  
- **이은혜 / 황다연**: FMS Adapter, PID 구현 지원, YOLO 기반 기능 개발

![팀 사진 또는 로고](./images/team_rotus.png)

---

## 📎 출처
- Demo Day Project Presentation (2025) :contentReference[oaicite:0]{index=0}

