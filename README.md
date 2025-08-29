# RoboCallee (Robot + Callee)
<p align="center">
  <img src="./docs/images/image72.png" alt="RoboCallee Logo" width="200"/>
</p>

---

## 1. 프로젝트 개요

스마트 매장에서 **로봇팔 + 자율 주행 로봇**을 활용해 신발 재고 운반과 고객 응대를 자동화하는 프로젝트입니다.  
고객 편의성 증대와 매장 운영 효율화를 동시에 달성하는 **로봇 기반 스마트 스토어**를 목표로 합니다.

### 프로젝트 수행 시나리오
![프로젝트 개요](./docs/images/image39.png)

### Map 구성
![프로젝트 개요](./docs/images/image46.png)

---

## 2. Git 폴더 구조

---

## 3. SW 아키텍처

![소프트웨어 아키텍처](./docs/images/image38.png)

---

## 4. FMS (Fleet Management System)

### FMS SW 아키텍처
![FMS SW 아키텍처](./docs/images/image36.png)

### 시나리오 수행 영상
| [![FMS 배달 시나리오](https://img.youtube.com/vi/GBW1qsYFHe4/0.jpg)](https://www.youtube.com/watch?v=GBW1qsYFHe4) | [![FMS 수거 시나리오](https://img.youtube.com/vi/zXWMFyJrSFo/0.jpg)](https://www.youtube.com/watch?v=zXWMFyJrSFo) | [![FMS 수거 시나리오](https://www.youtube.com/watch?v=zXWMFyJrSFo/0.jpg)](https://www.youtube.com/watch?v=zXWMFyJrSFo) |
|:----------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------:|
| FMS 배달 시나리오 | FMS 수거 시나리오 | FMS 수거 시나리오 GUI 요청 |

---

## 5. Control Service

### User GUI
| ![이미지1](./docs/images/image26.png) | ![이미지2](./docs/images/image27.png) | ![이미지3](./docs/images/image29.png) | ![이미지4](./docs/images/image30.png) |
|---------------------------------------------|---------------------------------------------|---------------------------------------------|---------------------------------------------|

### Admin GUI

#### Web GUI
![Control Service](./docs/images/image89.png)

#### QT GUI
![Control Service](./docs/images/image37.gif)

---

## 6. 로봇팔 제어

### 로봇팔 수행
| ![이미지1](./docs/images/image108.gif) | ![이미지2](./docs/images/image71.gif) | ![이미지3](./docs/images/image90.gif) | ![이미지4](./docs/images/image96.gif) |
|:--------------------------------------------:|:--------------------------------------------:|:--------------------------------------------:|:--------------------------------------------:|
| Shelf → Buffer                               | Buffer → Robot                               | Robot → Buffer                               | Buffer → Shelf                               |

### OCR
![이미지1](./docs/images/image91.gif)
 
---

## 7. 자율 주행 로봇
### PID 제어
![PID 주행](./docs/images/mobile_pid.gif)

### 강화학습 주행 (DQN)
![강화학습 주행](./docs/images/mobile_dqn.gif)

### Vision 기반 주행

#### vision tracker
| ![이미지1](./docs/images/image81.gif) | ![이미지2](./docs/images/image67.gif) |
|:--------------------------------------------:|:--------------------------------------------:|
| vision tracker                               | 로봇 시야                                    |

#### Obstacle avoidance
| ![이미지1](./docs/images/image87.gif) | ![이미지2](./docs/images/image93.gif) |
|:--------------------------------------------:|:--------------------------------------------:|
| 장애물 회피                               | 로봇 시야                                    |

---
