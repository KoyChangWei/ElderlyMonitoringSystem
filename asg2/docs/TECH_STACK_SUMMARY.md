# Elderly Care Dashboard - Tech Stack Quick Reference

## 🎯 System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    USER INTERFACES                               │
│  Web Dashboard (React) | Mobile App (React Native/Flutter)      │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                    BACKEND SERVICES                              │
│         Node.js + Express | Python FastAPI | Spring Boot        │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                      DATA LAYER                                  │
│  PostgreSQL | MongoDB | Redis | InfluxDB (Time-series)          │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                   AI/ML PROCESSING                               │
│  TensorFlow | PyTorch | OpenCV | YOLOv8 | GPT-4 API            │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                    IOT LAYER                                     │
│  MQTT | AWS IoT Core | Zigbee | Bluetooth LE | WiFi            │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                  HARDWARE DEVICES                                │
│  Wearables | Cameras | Sensors | Robot | Smart Home Devices    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📱 Frontend Stack

### Web Dashboard
| Technology | Purpose | Why? |
|-----------|---------|------|
| **React 18+** | UI Framework | Component-based, fast, large ecosystem |
| **TypeScript** | Type Safety | Catch errors early, better IDE support |
| **Redux Toolkit** | State Management | Predictable state, good DevTools |
| **Material-UI** | UI Components | Professional look, accessible, customizable |
| **Chart.js** | Data Visualization | Simple, performant, good for health charts |
| **Socket.io** | Real-time Updates | Live vital signs, instant alerts |
| **Axios** | HTTP Client | Promise-based, interceptors, easy to use |

### Mobile App
| Technology | Purpose | Why? |
|-----------|---------|------|
| **React Native** | Cross-platform | One codebase for iOS + Android |
| **Expo** | Development | Fast development, OTA updates |
| **React Navigation** | Routing | Native navigation feel |
| **Firebase** | Push Notifications | Reliable, free tier, easy setup |

---

## 🖥️ Backend Stack

### API Server
| Technology | Purpose | Why? |
|-----------|---------|------|
| **Node.js 18+** | Runtime | JavaScript everywhere, async, npm ecosystem |
| **Express.js** | Web Framework | Simple, flexible, middleware support |
| **TypeScript** | Language | Type safety, modern features |
| **JWT** | Authentication | Stateless, secure, mobile-friendly |
| **Joi / Zod** | Validation | Schema validation, type inference |

### Alternative: Python Stack
| Technology | Purpose | Why? |
|-----------|---------|------|
| **Python 3.11+** | Language | AI/ML libraries, readable, versatile |
| **FastAPI** | Framework | Fast, async, auto API docs, type hints |
| **Pydantic** | Validation | Data validation with Python type hints |
| **SQLAlchemy** | ORM | Database abstraction, migrations |

### Microservices
| Service | Technology | Port |
|---------|-----------|------|
| **API Gateway** | Kong / NGINX | 80/443 |
| **Auth Service** | Node.js + JWT | 3001 |
| **Health Service** | Python + FastAPI | 3002 |
| **IoT Service** | Node.js + MQTT | 3003 |
| **Robot Service** | Python + ROS2 | 3004 |
| **Notification Service** | Node.js + Bull | 3005 |

---

## 🗄️ Database Stack

### Relational Database
| Technology | Use Case | Data |
|-----------|----------|------|
| **PostgreSQL 15+** | Primary DB | Users, medications, medical records |
| **Extensions** | Advanced Features | PostGIS (location), TimescaleDB (time-series) |

### NoSQL Database
| Technology | Use Case | Data |
|-----------|----------|------|
| **MongoDB** | Document Store | IoT telemetry, logs, device configs |
| **InfluxDB** | Time-Series | Vital signs, sensor readings |
| **Redis** | Cache + Queue | Sessions, real-time data, pub/sub |

### Database Schema (PostgreSQL)
```sql
-- Key Tables
users (id, name, email, role, elderly_id)
elderly_profiles (id, age, medical_conditions, emergency_contacts)
health_records (id, elderly_id, timestamp, heart_rate, bp, spo2, temp)
medications (id, elderly_id, name, dosage, frequency, times)
medication_logs (id, medication_id, timestamp, taken, missed_reason)
devices (id, elderly_id, type, status, last_seen)
alerts (id, elderly_id, type, severity, timestamp, resolved)
```

---

## 🤖 AI/ML Stack

### Machine Learning Framework
| Technology | Use Case | Why? |
|-----------|----------|------|
| **TensorFlow 2.x** | Deep Learning | Production-ready, TF Lite for edge |
| **PyTorch** | Research/Training | Pythonic, flexible, great for experimentation |
| **Scikit-learn** | Traditional ML | Easy, comprehensive, well-documented |
| **XGBoost** | Gradient Boosting | Best for tabular data, medication adherence |

### Computer Vision
| Technology | Use Case | Why? |
|-----------|----------|------|
| **OpenCV** | Image Processing | Standard library, fast, comprehensive |
| **YOLOv8** | Object Detection | Real-time, accurate, easy to use |
| **MediaPipe** | Pose Estimation | Fall detection, lightweight, by Google |
| **TensorFlow Lite** | Edge Inference | Run on Jetson, Raspberry Pi |

### Natural Language Processing
| Technology | Use Case | Why? |
|-----------|----------|------|
| **OpenAI GPT-4** | Conversation | Best NLP, natural responses, context-aware |
| **Google Dialogflow** | Intent Recognition | Easy setup, voice integration |
| **Google Speech-to-Text** | Voice Input | Accurate, multiple languages |
| **Amazon Polly** | Voice Output | Natural voices, SSML support |

### ML Models Deployed
| Model | Algorithm | Accuracy | Latency |
|-------|-----------|----------|---------|
| **Health Prediction** | LSTM Neural Network | 92% | 50ms |
| **Fall Detection** | YOLOv8 + LSTM | 95% | 100ms |
| **Anomaly Detection** | Isolation Forest | 88% | 30ms |
| **Medication Adherence** | XGBoost | 91% | 20ms |

---

## 🌐 IoT Stack

### Communication Protocols
| Protocol | Use Case | Devices |
|---------|----------|---------|
| **MQTT** | Real-time messaging | All IoT devices |
| **WebSocket** | Dashboard updates | Web/mobile apps |
| **HTTP/REST** | Config, commands | API communication |
| **Bluetooth LE** | Wearables | Smartwatch, smart ring |
| **Zigbee 3.0** | Smart home | Lights, sensors, locks |
| **WiFi (2.4/5 GHz)** | High bandwidth | Cameras, robot |

### IoT Platform
| Technology | Purpose | Why? |
|-----------|---------|------|
| **AWS IoT Core** | Device Management | Scalable, secure, rules engine |
| **MQTT Broker (Mosquitto)** | Local messaging | Low latency, privacy |
| **Node-RED** | Automation | Visual, easy rules, local |
| **Home Assistant** | Smart home hub | 2000+ integrations, local control |

### Edge Computing
| Hardware | Specs | Use Case |
|---------|-------|----------|
| **NVIDIA Jetson Xavier NX** | 21 TOPS, 6W-15W | AI camera processing (4 cameras) |
| **Raspberry Pi 4 (8GB)** | Quad-core, 8GB RAM | Local hub, automation, Node-RED |
| **Intel NUC** | i5, 16GB RAM | Heavy processing, server |

---

## 🦾 Robot Stack

### Hardware Platform
| Component | Model | Price (RM) |
|-----------|-------|-----------|
| **Brain** | Raspberry Pi 4 (8GB) | 400 |
| **AI Processor** | NVIDIA Jetson Nano | 500 |
| **LIDAR** | RPLIDAR A1 | 1,000 |
| **Camera** | Raspberry Pi Camera v2 | 100 |
| **Chassis** | Mecanum wheel platform | 300 |
| **Display** | 7" touchscreen | 200 |
| **Battery** | 12V 20Ah Li-ion | 300 |
| **Gripper** | 2-finger servo gripper | 500 |
| **Sensors** | IMU, ultrasonic, IR | 200 |
| **Total** | | **~3,500** |

### Software Stack
| Technology | Purpose | Why? |
|-----------|---------|------|
| **Ubuntu 22.04** | OS | Stable, ROS2 support |
| **ROS2 Humble** | Robot Framework | Industry standard, modular |
| **Navigation2** | Autonomous Nav | Path planning, obstacle avoidance |
| **SLAM Toolbox** | Mapping | Create maps of home |
| **MoveIt2** | Arm Control | Motion planning for gripper |
| **OpenCV + YOLOv8** | Vision | Object detection, person tracking |
| **FastAPI** | Control API | Remote control from dashboard |

### Robot Capabilities
| Feature | Technology | Status |
|---------|-----------|--------|
| Autonomous Navigation | Nav2 + LIDAR | ✅ Ready |
| Obstacle Avoidance | Costmap2D | ✅ Ready |
| Voice Conversation | GPT-4 + TTS | ✅ Ready |
| Video Calling | WebRTC | ✅ Ready |
| Object Fetch | YOLOv8 + Gripper | ⚠️ Experimental |
| Medication Delivery | Custom dispenser | ✅ Ready |
| Fall Detection Response | Camera + AI | ✅ Ready |

---

## 🏠 IoT Hardware Devices

### Wearables (Per Elderly)
| Device | Model | Features | Price (RM) |
|--------|-------|----------|-----------|
| **Smartwatch** | Xiaomi Mi Band 8 | HR, SpO2, sleep, steps | 150 |
| **Smart Ring** | Oura Ring Gen 3 | Temp, HR, sleep quality | 1,300 |
| **Smart Shoes** | Custom ESP32 | Gait, pressure, fall detect | 250 |
| **Emergency Pendant** | Custom ESP32 + GPS | SOS button, GPS, GSM | 150 |

### Smart Home Sensors
| Device | Quantity | Model | Price (RM) |
|--------|---------|-------|-----------|
| **AI Cameras** | 4 | Wyze Cam v3 | 600 |
| **Smart Lights** | 6 | Xiaomi Yeelight | 300 |
| **Smart Lock** | 1 | August Smart Lock | 800 |
| **Temp/Humidity** | 4 | Xiaomi Sensor 2 | 120 |
| **Air Quality** | 1 | Xiaomi AQ Monitor | 150 |
| **Smoke Detector** | 2 | Nest Protect | 1,000 |
| **Smart Bed Sensor** | 1 | Withings Sleep | 500 |
| **AC Controller** | 1 | Sensibo Sky | 500 |
| **IoT Hub** | 1 | Raspberry Pi 4 | 400 |
| **TOTAL** | | | **~4,370** |

### Smart Medication Dispenser
| Component | Description | Price (RM) |
|-----------|-------------|-----------|
| **Brain** | Raspberry Pi 4 | 400 |
| **Servos** | 10x for compartments | 200 |
| **RTC Module** | Accurate timing | 20 |
| **Display** | LCD touchscreen | 50 |
| **Speaker** | Audio alerts | 30 |
| **Load Cells** | Pill detection | 100 |
| **Camera** | Verification | 100 |
| **Housing** | 3D printed | 200 |
| **TOTAL** | | **~1,100** |

**Total Hardware per Home**: **RM6,500 - 8,000** (without robot)
**With Robot**: **RM10,000 - 11,500**

---

## ☁️ Cloud Infrastructure (AWS)

### Core Services
| Service | Purpose | Cost/Month (100 patients) |
|---------|---------|--------------------------|
| **EC2** | API servers (2x t3.medium) | RM300 |
| **RDS** | PostgreSQL (db.t3.medium) | RM250 |
| **S3** | File storage (500GB) | RM45 |
| **IoT Core** | MQTT, device mgmt (10M msgs) | RM38 |
| **Lambda** | Serverless functions | RM50 |
| **SageMaker** | ML model hosting | RM100 |
| **CloudWatch** | Monitoring, logs | RM80 |
| **SNS/SES** | Notifications | RM70 |
| **TOTAL** | | **~RM833/month** |

### Scaling (1000 patients)
| Service | Configuration | Cost/Month |
|---------|--------------|-----------|
| **EC2** | 4x t3.large + Auto Scaling | RM1,200 |
| **RDS** | db.m5.xlarge Multi-AZ | RM800 |
| **S3** | 5TB storage | RM450 |
| **IoT Core** | 100M messages | RM375 |
| **Lambda** | 10M invocations | RM150 |
| **SageMaker** | Model hosting | RM500 |
| **Others** | CDN, monitoring, backups | RM500 |
| **TOTAL** | | **~RM3,975/month** |

---

## 🔐 Security Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| **Authentication** | JWT + OAuth 2.0 | Secure, stateless tokens |
| **Authorization** | RBAC (Role-Based) | Granular permissions |
| **Encryption (Transit)** | TLS 1.3 | Secure data transmission |
| **Encryption (Rest)** | AES-256 | Database, S3 encryption |
| **WAF** | AWS WAF / Cloudflare | Block attacks, SQL injection |
| **DDoS Protection** | AWS Shield | Prevent service disruption |
| **Secrets Management** | AWS Secrets Manager | API keys, credentials |
| **Monitoring** | CloudWatch + Sentry | Error tracking, alerts |
| **Compliance** | PDPA, ISO 27001 | Data protection standards |

---

## 📞 Communication Services

### Notifications
| Service | Use Case | Cost |
|---------|----------|------|
| **Firebase Cloud Messaging** | Push notifications | Free (1M/day) |
| **Twilio SMS** | Emergency alerts | RM0.30/SMS |
| **Twilio Voice** | Voice calls | RM0.50/min |
| **SendGrid** | Email notifications | RM0.10/1000 emails |
| **WhatsApp Business API** | WhatsApp messages | RM0.05/msg |

### Video Calling
| Service | Features | Cost |
|---------|----------|------|
| **Agora.io** | WebRTC, low latency | RM5/1000 min |
| **Twilio Video** | High quality | RM8/1000 min |
| **Jitsi Meet** | Open-source, self-hosted | Free (hosting cost) |

---

## 🔧 DevOps & Monitoring

### CI/CD Pipeline
```
GitHub → GitHub Actions → Docker Build → ECR → ECS Deploy
         ↓
    Automated Tests (Jest, Pytest, Cypress)
         ↓
    Security Scan (Snyk, OWASP)
         ↓
    Code Quality (SonarQube)
```

### Monitoring Stack
| Tool | Purpose | Why? |
|------|---------|------|
| **Prometheus** | Metrics collection | Open-source, powerful queries |
| **Grafana** | Dashboards | Beautiful visualizations |
| **ELK Stack** | Log aggregation | Elasticsearch, Logstash, Kibana |
| **Sentry** | Error tracking | Real-time alerts, stack traces |
| **New Relic** | APM | Application performance monitoring |
| **UptimeRobot** | Uptime monitoring | Service availability alerts |

---

## 💰 Cost Summary

### Initial Investment
| Category | Cost (RM) |
|---------|-----------|
| **Development Team (4 months)** | 372,000 |
| **Cloud & Tools** | 10,500 |
| **Pilot Hardware (10 homes)** | 80,000 |
| **Marketing & Legal** | 20,000 |
| **Contingency (10%)** | 48,250 |
| **TOTAL INITIAL INVESTMENT** | **~530,750** |

### Monthly Operating Costs (100 patients)
| Category | Cost (RM) |
|---------|-----------|
| **Cloud Infrastructure** | 1,433 |
| **Staff (Support, Nurses, IT)** | 30,000 |
| **Office & Utilities** | 3,000 |
| **Marketing** | 5,000 |
| **Insurance & Legal** | 2,000 |
| **TOTAL MONTHLY** | **41,433** |

### Revenue Projections
| Plan | Price | Margin | 100 Patients/Month |
|------|-------|--------|-------------------|
| **Basic** | RM299 | 72% | RM29,900 |
| **Premium** | RM599 | 85% | RM59,900 |
| **Complete** | RM999 | 88% | RM99,900 |
| **Mixed (Average)** | RM490 | 80% | RM49,000 |

**Break-even**: 85-140 patients (depending on plan mix)

---

## 📊 Key Performance Indicators

### Technical KPIs
- ✅ System Uptime: **>99.9%**
- ✅ API Response Time: **<200ms (p95)**
- ✅ Fall Detection Accuracy: **>95%**
- ✅ Alert Response Time: **<30 seconds**
- ✅ Mobile App Crash Rate: **<0.1%**

### Health KPIs
- ✅ Medication Adherence: **>95%**
- ✅ Fall Incident Reduction: **>50%**
- ✅ Hospital Readmission Reduction: **>30%**
- ✅ Emergency Response Time: **<5 minutes**

### Business KPIs
- ✅ Customer Satisfaction (NPS): **>70**
- ✅ Monthly Churn Rate: **<5%**
- ✅ Customer Lifetime Value: **>RM20,000**
- ✅ Customer Acquisition Cost: **<RM1,000**

---

## 🚀 Development Timeline

### Phase 1: MVP (Months 1-4)
```
Month 1: Foundation
  ✓ Backend API (Node.js + PostgreSQL)
  ✓ Frontend Dashboard (React)
  ✓ Authentication & User Management

Month 2: IoT Integration
  ✓ MQTT Broker Setup
  ✓ Wearable Device Integration
  ✓ Basic Health Monitoring
  ✓ Camera Feed

Month 3: AI Development
  ✓ Fall Detection Model
  ✓ Health Prediction Model
  ✓ Edge Deployment (Jetson)

Month 4: Testing & Launch
  ✓ User Testing
  ✓ Bug Fixes
  ✓ Documentation
  ✓ MVP Launch
```

### Phase 2: Advanced Features (Months 5-8)
```
Month 5-6: Robot Integration
  ✓ Robot Hardware Assembly
  ✓ ROS2 Setup & Navigation
  ✓ Remote Control Interface
  ✓ Voice Interaction

Month 7: Smart Home Expansion
  ✓ Smart Medication Dispenser
  ✓ IoT Device Expansion
  ✓ Automation Rules
  ✓ Nutrition Tracking

Month 8: Polish & Scale
  ✓ Mobile App Launch
  ✓ UI/UX Refinement
  ✓ Performance Optimization
  ✓ Production Deployment
```

---

## 📚 Technology Learning Resources

### Frontend Development
- **React**: https://react.dev/
- **TypeScript**: https://www.typescriptlang.org/docs/
- **Material-UI**: https://mui.com/

### Backend Development
- **Node.js**: https://nodejs.org/docs/
- **FastAPI**: https://fastapi.tiangolo.com/
- **PostgreSQL**: https://www.postgresql.org/docs/

### AI/ML
- **TensorFlow**: https://www.tensorflow.org/tutorials
- **PyTorch**: https://pytorch.org/tutorials/
- **YOLOv8**: https://docs.ultralytics.com/

### IoT & Robotics
- **ROS2**: https://docs.ros.org/en/humble/
- **AWS IoT**: https://docs.aws.amazon.com/iot/
- **MQTT**: https://mqtt.org/

### Cloud & DevOps
- **AWS**: https://aws.amazon.com/getting-started/
- **Docker**: https://docs.docker.com/
- **Kubernetes**: https://kubernetes.io/docs/

---

## 🛒 Where to Buy (Malaysia)

### IoT Components
- **Cytron Technologies**: https://www.cytron.io/ (Robotics, IoT)
- **MakerLab Electronics**: Sensors, Raspberry Pi, Arduino
- **RS Components Malaysia**: Industrial components
- **Shopee/Lazada**: Consumer smart home devices

### Robot Parts
- **RobotShop**: https://www.robotshop.com/ (International)
- **Cytron**: Local robotics supplier
- **AliExpress**: Budget components (longer shipping)

### Smart Home Devices
- **Xiaomi Malaysia**: Smart home ecosystem
- **Shopee/Lazada**: Various brands
- **Harvey Norman/Courts**: Premium brands

---

## 📞 Support & Community

### Malaysian Tech Community
- **IoT Association of Malaysia**: Networking, events
- **MDEC**: Government digital support
- **MaGIC**: Startup acceleration

### Developer Communities
- **Stack Overflow**: Q&A for all technologies
- **GitHub Discussions**: Open-source projects
- **Discord/Slack**: Real-time chat (ROS, React, AWS)

---

## ✅ Quick Start Checklist

### For Developers
- [ ] Install Node.js 18+, Python 3.11+
- [ ] Setup PostgreSQL, MongoDB, Redis locally
- [ ] Clone repository, install dependencies
- [ ] Configure environment variables (.env)
- [ ] Run database migrations
- [ ] Start backend server
- [ ] Start frontend dev server
- [ ] Access dashboard at http://localhost:3000

### For IoT Setup
- [ ] Purchase hardware devices
- [ ] Setup MQTT broker (Mosquitto)
- [ ] Configure AWS IoT Core
- [ ] Register devices, create certificates
- [ ] Flash firmware to ESP32/Arduino devices
- [ ] Test MQTT connectivity
- [ ] Integrate with dashboard

### For Robot Setup
- [ ] Assemble robot hardware
- [ ] Install Ubuntu 22.04 on Raspberry Pi
- [ ] Install ROS2 Humble
- [ ] Clone robot software packages
- [ ] Configure network, sensors
- [ ] Test navigation, camera
- [ ] Integrate with dashboard API

---

**Document Version**: 1.0
**Last Updated**: November 25, 2025
**Quick Reference for**: Developers, Project Managers, Stakeholders
