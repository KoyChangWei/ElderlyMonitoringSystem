# Elderly Care Dashboard - Quick Reference Guide

## 📚 Documentation Overview

Your dashboard now has **COMPLETE** technical documentation:

### 1. **FUNCTIONAL_SPECIFICATION.md** (Part 1)
- ✅ Feature 1: Real-Time Clock
- ✅ Feature 2: Elderly Profile Display
- ✅ Feature 3: System Alert Status
- ✅ Feature 4: Emergency SOS Button
- ✅ Feature 5: Real-Time Vital Signs (4 metrics)
- ✅ Feature 6: AI Health Prediction
- ✅ Feature 7: Robot Caregiver Status & Control

**Each feature includes**:
- Purpose & use case
- Frontend implementation code
- Backend architecture
- API endpoints with examples
- Database schema
- Data flow diagrams
- Tech stack details
- Scope & future enhancements

### 2. **FUNCTIONAL_SPECIFICATION_PART2.md**
- ✅ Feature 8: Activity & Safety Monitoring
- ✅ Features 9-14: IoT Smart Home (6 devices)
- ✅ Feature 15: Health Trends Chart

**Detailed implementations for**:
- Steps, sleep, fall detection, mood tracking
- Smart lights, AC, cameras, door lock, air quality, voice assistant
- Chart.js visualizations with real-time updates

### 3. **COMPLETE_SYSTEM_ARCHITECTURE.md**
- ✅ Full system architecture diagram
- ✅ Complete database schema (28 tables)
- ✅ All 85+ API endpoints
- ✅ Cost analysis & revenue model
- ✅ Infrastructure specifications

### 4. **TECHNICAL_PROPOSAL.md** (25+ pages)
- Complete tech stack breakdown
- Hardware specifications with Malaysia prices
- Cloud infrastructure details
- AI/ML model specifications
- Robot implementation
- Security & compliance

### 5. **TECH_STACK_SUMMARY.md**
- Quick reference tables
- Technology comparisons
- Hardware device list with prices
- Where to buy in Malaysia

### 6. **IMPLEMENTATION_GUIDE.md**
- Step-by-step setup instructions
- Code examples for all layers
- Docker deployment
- AWS configuration
- Testing strategies

---

## 🎯 Feature-to-Tech Mapping

### Frontend Features → Backend Services

| Dashboard Feature | Backend Service | Tech Stack | Database |
|------------------|----------------|-----------|----------|
| Real-time Clock | Time Sync (optional) | JavaScript Date API | None |
| Elderly Profile | User Profile Service | Node.js + Express | PostgreSQL (elderly_profiles) |
| Alert Status | Alert Aggregation Service | Node.js + Redis Pub/Sub | PostgreSQL (alerts) |
| Emergency SOS | Emergency Response Service | Node.js + Twilio + FCM | PostgreSQL (emergencies) |
| Vital Signs | Health Monitoring Service | Python + FastAPI | InfluxDB + PostgreSQL |
| AI Prediction | AI/ML Service | Python + TensorFlow | PostgreSQL (predictions) |
| Robot Status | Robot Control Service | Python + ROS2 | PostgreSQL (robots) |
| Activity Monitor | Activity Service | Node.js + InfluxDB | PostgreSQL + InfluxDB |
| IoT Devices (6) | IoT Management Service | Node.js + MQTT | PostgreSQL (iot_devices) |
| Health Chart | Analytics Service | Python + Pandas | InfluxDB |
| Medication | Medication Service | Node.js + Scheduler | PostgreSQL (medications) |
| AI Insights | AI Service + Analytics | Python + ML models | PostgreSQL |
| Emergency Contacts | Emergency Service | Node.js | PostgreSQL |
| Activity Feed | Activity Service | Node.js + MongoDB | MongoDB + PostgreSQL |

---

## 🏗️ System Architecture (Simple View)

```
USER (Browser/Mobile)
    ↓ HTTPS
API Gateway (Kong) → Load Balancer
    ↓
┌─────────────────────────────────────────┐
│     Microservices (9 services)          │
│  Node.js / Python / Go                  │
└─────────────────┬───────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│         Databases                        │
│  PostgreSQL | InfluxDB | MongoDB | Redis│
└─────────────────┬───────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│      IoT Layer                           │
│  MQTT Broker ← IoT Devices               │
│  Wearables | Cameras | Sensors | Robot   │
└─────────────────────────────────────────┘
```

---

## 📊 Database Tables (28 Total)

### Core Tables (8)
1. **users** - Caregivers, family, doctors
2. **elderly_profiles** - Elderly patient data
3. **user_elderly_access** - Access control
4. **health_records** - Vital signs history
5. **medications** - Prescriptions
6. **medication_logs** - Adherence tracking
7. **iot_devices** - Device registry
8. **alerts** - System alerts

### Advanced Tables (20)
9. health_predictions
10. device_commands
11. automation_rules
12. robots
13. robot_commands
14. robot_tasks
15. daily_activity
16. fall_incidents
17. sleep_records
18. notifications
19. emergencies
20. emergency_actions
21. emergency_contacts
22. mood_records
23. rule_executions
24. prediction_outcomes
25. audit_log
26. system_config
27. session_tokens
28. device_telemetry (InfluxDB)

---

## 🔌 Key API Endpoints

### Health Monitoring
```javascript
GET  /api/health/:elderlyId/vitals/latest
POST /api/health/:elderlyId/vitals
GET  /api/health/:elderlyId/trends?metric=heartRate&hours=24
```

### AI Predictions
```javascript
GET /api/ai/health-prediction/:elderlyId
// Response: { stabilityScore: 95, confidence: 0.92, ... }
```

### Robot Control
```javascript
GET  /api/robot/:elderlyId/status
POST /api/robot/:elderlyId/command
// Body: { action: "move_forward", parameters: {...} }
```

### IoT Devices
```javascript
GET  /api/iot/:elderlyId/devices
POST /api/iot/device/control
// Body: { deviceId: "LIGHT-001", action: "on" }
```

### Emergency
```javascript
POST /api/emergency/sos
// Body: { elderlyId: 1, location: {...} }
// Response: { emergencyId: "EMG-001", actions: [...] }
```

### Medications
```javascript
GET /api/medications/:elderlyId/schedule/today
POST /api/medications/logs
// Body: { medicationId: 1, status: "taken", actualTime: "..." }
```

---

## 💻 Tech Stack Summary

### Frontend
- **Framework**: React 18 + TypeScript
- **UI**: Material-UI / Ant Design
- **Charts**: Chart.js
- **Real-time**: Socket.io client
- **State**: Redux Toolkit

### Backend
- **Primary**: Node.js 18 + Express.js
- **AI/ML**: Python 3.11 + FastAPI
- **Robot**: Python + ROS2 Humble

### Databases
- **PostgreSQL 15**: Structured data (28 tables)
- **InfluxDB**: Time-series (vital signs)
- **MongoDB**: Logs & unstructured data
- **Redis 7**: Cache & real-time state

### AI/ML
- **TensorFlow 2.x**: Health prediction LSTM
- **YOLOv8**: Fall detection (95% accuracy)
- **OpenCV**: Computer vision
- **OpenAI GPT-4**: Robot conversation

### IoT
- **Protocol**: MQTT (Mosquitto broker)
- **Platform**: AWS IoT Core
- **Devices**: 15+ per home
- **Edge**: NVIDIA Jetson Xavier NX

### Cloud (AWS)
- **Compute**: EC2, Lambda
- **Database**: RDS PostgreSQL
- **IoT**: AWS IoT Core
- **AI**: SageMaker
- **Storage**: S3

---

## 💰 Cost Breakdown

### Development (One-time)
- Team (4 months): **RM 372,000**
- Tools & infrastructure: **RM 10,500**
- **Total**: **RM 382,500**

### Hardware (Per Home)
- Wearables: RM 1,850
- Smart home devices: RM 4,370
- Medication dispenser: RM 1,100
- Robot (optional): RM 3,500
- **Total**: **RM 8,000 - 11,500**

### Monthly Operations (100 patients)
- Cloud infrastructure: RM 1,283
- Third-party APIs: RM 1,050
- Staff (nurses, support, IT): RM 30,000
- Office & utilities: RM 3,000
- Insurance & legal: RM 2,000
- Marketing: RM 5,000
- **Total**: **RM 42,333/month**
- **Per patient**: **RM 423/month**

### Revenue Model
- Basic Plan: **RM 299/month**
- Premium Plan: **RM 599/month**
- Complete Plan: **RM 999/month**

**Break-even**: 87-142 patients (depending on plan mix)

---

## 🚀 Implementation Timeline

### Phase 1: MVP (4 months)
- Month 1: Backend + Frontend foundation
- Month 2: IoT integration + Health monitoring
- Month 3: AI models + Edge deployment
- Month 4: Testing + MVP launch

### Phase 2: Advanced Features (4 months)
- Months 5-6: Robot integration
- Month 7: Smart home expansion + Medication system
- Month 8: Polish + Production deployment

### Total Time to Full Platform: **8 months**

---

## 🔐 Security Features

- ✅ JWT authentication (24-hour expiry)
- ✅ TLS 1.3 encryption (all communications)
- ✅ AES-256 database encryption
- ✅ Role-based access control (RBAC)
- ✅ Multi-factor authentication (MFA)
- ✅ AWS WAF + DDoS protection
- ✅ PDPA compliance (Malaysia)
- ✅ Audit logging (all actions)

---

## 📈 Performance Targets

- ✅ System uptime: **>99.9%**
- ✅ API response time: **<200ms (p95)**
- ✅ Real-time updates: **<1 second latency**
- ✅ Fall detection: **<500ms alert time**
- ✅ Emergency response: **<5 minutes**
- ✅ Concurrent users: **10,000+**
- ✅ IoT devices supported: **100,000+**

---

## 📞 Support & Resources

### Documentation Files
1. `FUNCTIONAL_SPECIFICATION.md` (Part 1)
2. `FUNCTIONAL_SPECIFICATION_PART2.md`
3. `COMPLETE_SYSTEM_ARCHITECTURE.md`
4. `TECHNICAL_PROPOSAL.md`
5. `TECH_STACK_SUMMARY.md`
6. `IMPLEMENTATION_GUIDE.md`
7. `README.md`

### Hardware Suppliers (Malaysia)
- **Cytron Technologies**: Robotics, IoT components
- **MakerLab Electronics**: Raspberry Pi, sensors
- **Shopee/Lazada**: Smart home devices
- **RS Components Malaysia**: Industrial IoT

### Learning Resources
- React: https://react.dev/
- Node.js: https://nodejs.org/docs/
- ROS2: https://docs.ros.org/en/humble/
- TensorFlow: https://www.tensorflow.org/
- AWS IoT: https://docs.aws.amazon.com/iot/

---

## ✅ What You Have Now

### For Your Assignment
- ✅ Complete HTML/CSS/JS prototype (working dashboard)
- ✅ Gestalt principles documentation
- ✅ Human factors engineering analysis
- ✅ Comprehensive technical report

### For Implementation
- ✅ Complete backend architecture
- ✅ All API specifications (85+ endpoints)
- ✅ Full database schema (28 tables)
- ✅ Tech stack with alternatives
- ✅ Cost analysis & revenue model
- ✅ Hardware list with prices (Malaysia)
- ✅ Step-by-step implementation guide

### For Investors/Stakeholders
- ✅ Technical proposal (25+ pages)
- ✅ Market analysis
- ✅ Competitive advantages
- ✅ Scalability plan
- ✅ ROI calculations

---

## 🎓 Assignment Submission Checklist

For your Human Factor Engineering assignment:

- ✅ **Prototype**: index.html (main dashboard) + 5 detail pages
- ✅ **Design Documentation**: Gestalt principles applied
- ✅ **Technical Report**: Complete system architecture
- ✅ **Innovation**: 10 features not common in Malaysia
- ✅ **User-Centered Design**: Intuitive for all literacy levels
- ✅ **Cognition Support**: Memory aids, clear navigation
- ✅ **Real-world Feasibility**: Complete tech stack documented

---

**All documentation is ready for your assignment and real-world implementation!** 🎉

Last Updated: November 25, 2025
Course: STTHK3133 Human Factor Engineering
Universiti Utara Malaysia (UUM)
