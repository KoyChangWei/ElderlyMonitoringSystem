# Elderly Care Monitoring Dashboard
# Complete Functional Specification & Backend Architecture

**Document Version**: 1.0
**Date**: November 25, 2025
**Project**: Elderly Care Monitoring Dashboard
**Course**: STTHK3133 Human Factor Engineering

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Feature-by-Feature Specification](#2-feature-by-feature-specification)
3. [Complete Backend Architecture](#3-complete-backend-architecture)
4. [Database Schema Design](#4-database-schema-design)
5. [API Specifications](#5-api-specifications)
6. [Tech Stack Breakdown](#6-tech-stack-breakdown)
7. [Data Flow Diagrams](#7-data-flow-diagrams)
8. [Security Implementation](#8-security-implementation)
9. [Scalability & Performance](#9-scalability--performance)

---

## 1. System Overview

### 1.1 Architecture Layers
```
┌─────────────────────────────────────────────────────────────────┐
│                     PRESENTATION LAYER                          │
│  Web Dashboard (React) | Mobile App (React Native)              │
│  - Real-time UI updates via WebSocket                           │
│  - Responsive design for all devices                            │
└────────────────────────┬────────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────────┐
│                    APPLICATION LAYER                             │
│  API Gateway (Kong/NGINX) → Microservices                       │
│  - Load Balancing | Rate Limiting | Authentication              │
└────────────────────────┬────────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────────┐
│                    BUSINESS LOGIC LAYER                          │
│  Microservices (Node.js/Python/Go)                              │
│  - Auth Service | Health Service | IoT Service                  │
│  - Robot Service | Notification Service | AI Service            │
└────────────────────────┬────────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────────┐
│                     DATA LAYER                                   │
│  PostgreSQL (Relational) | MongoDB (NoSQL)                      │
│  Redis (Cache) | InfluxDB (Time-Series)                         │
└────────────────────────┬────────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────────┐
│                    IOT & INTEGRATION LAYER                       │
│  MQTT Broker | AWS IoT Core | Device Firmware                   │
│  - Wearables | Cameras | Sensors | Robot                        │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Feature-by-Feature Specification

### FEATURE 1: Real-Time Clock Display

**Frontend Location**: `Header > #currentTime`

#### Purpose
- Display current date and time in Malaysian format
- Updates every second for accuracy
- Provides temporal context for caregivers

#### Frontend Implementation
```javascript
// JavaScript (script.js)
function updateClock() {
  const now = new Date();
  const options = {
    weekday: 'short',
    year: 'numeric',
    month: 'short',
    day: 'numeric',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    timeZone: 'Asia/Kuala_Lumpur'
  };
  document.getElementById('currentTime').textContent =
    now.toLocaleDateString('en-MY', options);
}

setInterval(updateClock, 1000);
```

#### Backend Architecture
**Service**: Time Sync Service (Optional for server time)

**Tech Stack**:
- **Frontend**: Vanilla JavaScript Date API
- **Backend**: NTP (Network Time Protocol) server for time synchronization
- **Alternative**: AWS Time Sync Service

**API Endpoint**:
```
GET /api/system/time
Response: { "timestamp": "2025-11-25T14:30:00+08:00", "timezone": "Asia/Kuala_Lumpur" }
```

**Database**: Not required (handled client-side)

**Scope**:
- ✅ Client-side clock (implemented)
- ⚠️ Server time sync for accuracy
- ⚠️ Timezone management for multi-region

---

### FEATURE 2: Elderly Profile Information

**Frontend Location**: `Header > .elderly-info`

#### Purpose
- Display elderly person's basic information
- Quick identification for multi-patient monitoring
- Avatar for visual recognition

#### Frontend Implementation
```html
<div class="elderly-info">
  <img src="avatar-url" class="elder-avatar">
  <div>
    <h2>Ah Chong Tan</h2>
    <p>Age: 78 | Room: A-203</p>
  </div>
</div>
```

#### Backend Architecture

**Service**: User Profile Service

**Tech Stack**:
- **Backend**: Node.js + Express.js
- **Database**: PostgreSQL
- **Image Storage**: AWS S3 / Cloudinary
- **CDN**: CloudFront for avatar delivery

**API Endpoints**:
```javascript
// Get elderly profile
GET /api/elderly/:elderlyId/profile
Response: {
  "id": 1,
  "name": "Ah Chong Tan",
  "age": 78,
  "dateOfBirth": "1947-03-15",
  "gender": "male",
  "room": "A-203",
  "avatarUrl": "https://cdn.eldercare.com/avatars/001.jpg",
  "medicalConditions": ["Hypertension", "Type 2 Diabetes"],
  "allergies": ["Penicillin"],
  "emergencyContacts": [...]
}

// Update profile
PUT /api/elderly/:elderlyId/profile
Body: { "name": "...", "age": 78, ... }
Response: { "success": true, "updated": {...} }

// Upload avatar
POST /api/elderly/:elderlyId/avatar
Content-Type: multipart/form-data
Body: { "avatar": [file] }
Response: { "avatarUrl": "https://..." }
```

**Database Schema**:
```sql
CREATE TABLE elderly_profiles (
  id SERIAL PRIMARY KEY,
  user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
  name VARCHAR(255) NOT NULL,
  date_of_birth DATE NOT NULL,
  age INTEGER GENERATED ALWAYS AS (
    EXTRACT(YEAR FROM AGE(CURRENT_DATE, date_of_birth))
  ) STORED,
  gender VARCHAR(20),
  room_number VARCHAR(50),
  avatar_url TEXT,
  medical_conditions TEXT[],
  allergies TEXT[],
  blood_type VARCHAR(10),
  height DECIMAL(5,2), -- in cm
  weight DECIMAL(5,2), -- in kg
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for fast lookups
CREATE INDEX idx_elderly_user_id ON elderly_profiles(user_id);
```

**Data Flow**:
1. Dashboard loads → Fetch profile data via API
2. Profile service queries PostgreSQL
3. Avatar URL retrieved from S3
4. Data cached in Redis (15-minute TTL)
5. Real-time updates via WebSocket when profile changes

**Scope**:
- ✅ Basic profile display
- ⚠️ Profile editing interface
- ⚠️ Medical history integration
- ⚠️ Multi-language name support

---

### FEATURE 3: System Alert Status

**Frontend Location**: `Header > #alertStatus`

#### Purpose
- Display overall system health status
- Immediate visual feedback for caregivers
- Color-coded for quick recognition (green=normal, yellow=warning, red=critical)

#### Frontend Implementation
```javascript
// Update alert status
function updateAlertStatus(status) {
  const alertStatus = document.getElementById('alertStatus');

  const statuses = {
    normal: {
      icon: 'fa-check-circle',
      text: 'All Systems Normal',
      color: '#27AE60'
    },
    warning: {
      icon: 'fa-exclamation-triangle',
      text: 'Warnings Detected',
      color: '#F39C12'
    },
    critical: {
      icon: 'fa-times-circle',
      text: 'Critical Alert!',
      color: '#E74C3C'
    }
  };

  const config = statuses[status];
  alertStatus.style.background = `rgba(${hexToRgb(config.color)}, 0.3)`;
  alertStatus.innerHTML = `
    <i class="fas ${config.icon}"></i>
    <span>${config.text}</span>
  `;
}
```

#### Backend Architecture

**Service**: Alert Aggregation Service

**Tech Stack**:
- **Backend**: Node.js + Express.js
- **Message Queue**: Redis Pub/Sub / Apache Kafka
- **Real-time**: Socket.io / Server-Sent Events
- **Database**: PostgreSQL + Redis

**API Endpoints**:
```javascript
// Get current system status
GET /api/alerts/status/:elderlyId
Response: {
  "status": "normal" | "warning" | "critical",
  "activeAlerts": 0,
  "criticalAlerts": 0,
  "lastChecked": "2025-11-25T14:30:00Z",
  "systems": {
    "health": "normal",
    "iot": "normal",
    "robot": "normal",
    "medication": "warning"
  }
}

// Subscribe to real-time status updates
WebSocket: ws://api.eldercare.com/ws/alerts/:elderlyId
Message: {
  "type": "status_change",
  "oldStatus": "normal",
  "newStatus": "warning",
  "reason": "Vital signs abnormal"
}
```

**Alert Rules Engine**:
```javascript
// Alert Rules (pseudocode)
const alertRules = [
  {
    name: "Critical Vital Signs",
    condition: (data) =>
      data.heartRate > 120 || data.heartRate < 50 ||
      data.bloodPressureSys > 180 || data.bloodPressureSys < 80,
    severity: "critical"
  },
  {
    name: "Missed Medication",
    condition: (data) =>
      data.timeSinceDue > 30, // minutes
    severity: "warning"
  },
  {
    name: "Fall Detected",
    condition: (data) =>
      data.fallDetected === true,
    severity: "critical"
  },
  {
    name: "Low Device Battery",
    condition: (data) =>
      data.batteryLevel < 20,
    severity: "warning"
  }
];

// Processing pipeline
function processAlerts(elderlyId) {
  const data = gatherAllSensorData(elderlyId);
  const alerts = alertRules
    .filter(rule => rule.condition(data))
    .map(rule => ({ ...rule, elderlyId, timestamp: new Date() }));

  if (alerts.length > 0) {
    const maxSeverity = getMaxSeverity(alerts);
    publishStatusUpdate(elderlyId, maxSeverity);
    notifyCaregivers(elderlyId, alerts);
  }
}
```

**Database Schema**:
```sql
CREATE TABLE alerts (
  id SERIAL PRIMARY KEY,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  alert_type VARCHAR(100) NOT NULL,
  severity VARCHAR(20) NOT NULL, -- critical, warning, info
  title VARCHAR(255) NOT NULL,
  description TEXT,
  source VARCHAR(100), -- health, iot, robot, medication
  triggered_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  acknowledged_at TIMESTAMP,
  acknowledged_by INTEGER REFERENCES users(id),
  resolved_at TIMESTAMP,
  resolved_by INTEGER REFERENCES users(id),
  metadata JSONB, -- Additional context
  is_active BOOLEAN DEFAULT TRUE
);

CREATE INDEX idx_alerts_elderly_active ON alerts(elderly_id, is_active);
CREATE INDEX idx_alerts_severity ON alerts(severity, triggered_at);
```

**Data Flow**:
1. Sensors/devices send data → MQTT broker
2. Alert service subscribes to sensor topics
3. Rules engine evaluates conditions
4. Alert generated → Saved to PostgreSQL
5. Status aggregated across all systems
6. WebSocket pushes update to dashboard
7. Dashboard updates UI immediately

**Scope**:
- ✅ Real-time status display
- ⚠️ Alert history viewer
- ⚠️ Custom alert rules configuration
- ⚠️ Alert acknowledgment workflow
- ⚠️ Alert escalation (if unacknowledged)

---

### FEATURE 4: Emergency SOS Button

**Frontend Location**: `Header > .emergency-btn`

#### Purpose
- One-click emergency alert activation
- Immediate notification to all stakeholders
- Triggers multiple emergency protocols
- Large, prominent button for accessibility

#### Frontend Implementation
```javascript
function triggerEmergency() {
  // Confirmation dialog
  const confirmed = confirm(
    '🚨 EMERGENCY ALERT\n\n' +
    'This will:\n' +
    '- Call emergency services (999)\n' +
    '- Alert all family members\n' +
    '- Activate all cameras\n' +
    '- Send location to responders\n\n' +
    'Proceed?'
  );

  if (confirmed) {
    // Disable button to prevent double-trigger
    const btn = event.target;
    btn.disabled = true;
    btn.textContent = 'CALLING EMERGENCY...';

    // Send emergency request
    fetch('/api/emergency/sos', {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        elderlyId: currentElderlyId,
        triggeredBy: 'caregiver',
        location: await getCurrentLocation(),
        timestamp: new Date().toISOString()
      })
    })
    .then(response => response.json())
    .then(data => {
      showNotification('Emergency services activated! Help is on the way.', 'critical');
      updateAlertStatus('critical');
    })
    .catch(error => {
      showNotification('Error activating emergency! Please call 999 manually.', 'error');
      btn.disabled = false;
    });
  }
}
```

#### Backend Architecture

**Service**: Emergency Response Service

**Tech Stack**:
- **Backend**: Node.js + Express.js (High availability setup)
- **Message Queue**: RabbitMQ / AWS SQS (for guaranteed delivery)
- **SMS/Call API**: Twilio / AWS SNS
- **Push Notifications**: Firebase Cloud Messaging
- **Email**: SendGrid / AWS SES
- **Database**: PostgreSQL (for logging) + Redis (for coordination)

**API Endpoints**:
```javascript
// Trigger emergency SOS
POST /api/emergency/sos
Authorization: Bearer <token>
Body: {
  "elderlyId": 1,
  "triggeredBy": "caregiver" | "elderly" | "system",
  "location": {
    "latitude": 3.1390,
    "longitude": 101.6869,
    "address": "Kuala Lumpur"
  },
  "notes": "Optional notes"
}
Response: {
  "emergencyId": "EMG-2025-001",
  "status": "activated",
  "activatedAt": "2025-11-25T14:30:00Z",
  "actions": [
    { "type": "call_999", "status": "initiated" },
    { "type": "notify_family", "status": "sent", "recipients": 3 },
    { "type": "unlock_doors", "status": "completed" },
    { "type": "activate_cameras", "status": "completed" }
  ]
}

// Get emergency status
GET /api/emergency/:emergencyId/status
Response: {
  "emergencyId": "EMG-2025-001",
  "status": "active" | "resolved" | "cancelled",
  "responders": [
    { "name": "Ambulance 123", "eta": "5 minutes", "status": "enroute" }
  ],
  "timeline": [...]
}

// Cancel false alarm
POST /api/emergency/:emergencyId/cancel
Body: { "reason": "false alarm", "cancelledBy": userId }
Response: { "success": true }
```

**Emergency Protocol Workflow**:
```javascript
// Emergency Service Implementation
class EmergencyService {
  async activateSOS(data) {
    const emergencyId = generateEmergencyId();

    // 1. Log to database (critical - must not fail)
    await this.logEmergency(emergencyId, data);

    // 2. Execute emergency actions in parallel
    const actions = await Promise.allSettled([
      this.callEmergencyServices(data),
      this.notifyFamily(data.elderlyId),
      this.notifyNurses(data.elderlyId),
      this.unlockDoors(data.elderlyId),
      this.activateAllCameras(data.elderlyId),
      this.sendLocationToResponders(data.location),
      this.activateRobotEmergencyMode(data.elderlyId),
      this.turnOnAllLights(data.elderlyId)
    ]);

    // 3. Send status update via WebSocket
    this.broadcastEmergencyStatus(emergencyId, actions);

    // 4. Start monitoring emergency resolution
    this.monitorEmergencyStatus(emergencyId);

    return { emergencyId, actions };
  }

  async callEmergencyServices(data) {
    // Integration with Twilio Voice API
    const call = await twilioClient.calls.create({
      to: '+60999', // Emergency number
      from: process.env.TWILIO_NUMBER,
      twiml: `
        <Response>
          <Say voice="alice">
            Emergency alert for elderly patient ${data.elderlyName}.
            Location: ${data.location.address}.
            Coordinates: ${data.location.latitude}, ${data.location.longitude}.
          </Say>
          <Dial>${data.emergencyContacts[0].phone}</Dial>
        </Response>
      `
    });

    return { callSid: call.sid, status: 'initiated' };
  }

  async notifyFamily(elderlyId) {
    const family = await this.getFamilyContacts(elderlyId);

    const notifications = await Promise.all([
      // SMS
      family.map(member =>
        twilioClient.messages.create({
          to: member.phone,
          from: process.env.TWILIO_NUMBER,
          body: `🚨 EMERGENCY: ${elderlyName} has triggered an emergency alert. Emergency services have been notified. Location: ${location}`
        })
      ),
      // Push notification
      this.sendPushNotification(family, {
        title: '🚨 Emergency Alert',
        body: `${elderlyName} needs immediate assistance`,
        data: { elderlyId, emergencyId }
      }),
      // Email
      this.sendEmergencyEmail(family, data)
    ]);

    return { notified: family.length, methods: ['sms', 'push', 'email'] };
  }

  async unlockDoors(elderlyId) {
    // Send command to IoT door locks
    await mqttClient.publish(
      `elderly/${elderlyId}/iot/door/command`,
      JSON.stringify({ action: 'unlock', reason: 'emergency' })
    );

    return { status: 'unlocked' };
  }

  async activateAllCameras(elderlyId) {
    // Turn on all cameras and start recording
    await mqttClient.publish(
      `elderly/${elderlyId}/iot/cameras/command`,
      JSON.stringify({ action: 'record', quality: 'high' })
    );

    // Stream to emergency dashboard
    return { streaming: true, cameras: 4 };
  }
}
```

**Database Schema**:
```sql
CREATE TABLE emergencies (
  id SERIAL PRIMARY KEY,
  emergency_id VARCHAR(50) UNIQUE NOT NULL,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  triggered_by INTEGER REFERENCES users(id),
  trigger_source VARCHAR(50), -- caregiver, elderly_button, system_auto
  status VARCHAR(50) DEFAULT 'active', -- active, resolved, cancelled
  severity VARCHAR(20) DEFAULT 'critical',
  location_lat DECIMAL(10, 8),
  location_lng DECIMAL(11, 8),
  location_address TEXT,
  notes TEXT,
  activated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  resolved_at TIMESTAMP,
  resolved_by INTEGER REFERENCES users(id),
  resolution_notes TEXT,
  false_alarm BOOLEAN DEFAULT FALSE
);

CREATE TABLE emergency_actions (
  id SERIAL PRIMARY KEY,
  emergency_id INTEGER REFERENCES emergencies(id),
  action_type VARCHAR(100), -- call_999, notify_family, unlock_doors, etc.
  action_status VARCHAR(50), -- initiated, completed, failed
  started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  completed_at TIMESTAMP,
  result JSONB,
  error_message TEXT
);

CREATE TABLE emergency_timeline (
  id SERIAL PRIMARY KEY,
  emergency_id INTEGER REFERENCES emergencies(id),
  timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  event_type VARCHAR(100),
  description TEXT,
  actor INTEGER REFERENCES users(id),
  metadata JSONB
);

-- Indexes
CREATE INDEX idx_emergencies_status ON emergencies(status, activated_at);
CREATE INDEX idx_emergencies_elderly ON emergencies(elderly_id);
```

**Data Flow**:
```
1. User clicks SOS button → POST /api/emergency/sos
2. Emergency service receives request
3. Generate unique emergency ID
4. Log to database (PostgreSQL)
5. Execute parallel actions:
   ├─ Call 999 via Twilio
   ├─ SMS to family via Twilio
   ├─ Push notifications via FCM
   ├─ Email via SendGrid
   ├─ MQTT command to unlock doors
   ├─ MQTT command to activate cameras
   ├─ Robot emergency mode activation
   └─ Turn on all lights
6. Broadcast status via WebSocket
7. Dashboard updates in real-time
8. Monitor resolution and log timeline
```

**Scope**:
- ✅ One-click emergency activation
- ✅ Multi-channel notifications (SMS, push, email, call)
- ✅ Automated IoT responses
- ⚠️ Integration with local emergency services API
- ⚠️ GPS tracking during emergency
- ⚠️ Emergency timeline visualization
- ⚠️ False alarm handling
- ⚠️ Emergency drill mode (testing without actual calls)

---

### FEATURE 5: Real-Time Vital Signs Monitoring

**Frontend Location**: `.status-card.health-status > .vital-signs`

#### Purpose
- Display 4 critical vital signs in real-time
- Heart rate, blood pressure, oxygen saturation, temperature
- Color-coded status indicators (normal, warning, critical)
- Click to navigate to detailed health analytics

#### Frontend Implementation
```javascript
// WebSocket connection for real-time updates
const socket = io('wss://api.eldercare.com', {
  auth: { token: localStorage.getItem('token') }
});

socket.on('vital_signs_update', (data) => {
  updateVitalSigns(data);
});

function updateVitalSigns(data) {
  // Heart Rate
  document.getElementById('heartRate').textContent = `${data.heartRate} bpm`;
  updateStatusIndicator('heartRate', getHealthStatus(data.heartRate, 60, 100));

  // Blood Pressure
  document.getElementById('bloodPressure').textContent =
    `${data.bloodPressure.systolic}/${data.bloodPressure.diastolic}`;
  updateStatusIndicator('bloodPressure',
    getHealthStatus(data.bloodPressure.systolic, 90, 140));

  // O2 Saturation
  document.getElementById('oxygen').textContent = `${data.spo2}%`;
  updateStatusIndicator('oxygen', getHealthStatus(data.spo2, 95, 100));

  // Temperature
  document.getElementById('temperature').textContent = `${data.temperature}°C`;
  updateStatusIndicator('temperature', getHealthStatus(data.temperature, 36.1, 37.2));
}

function getHealthStatus(value, normalMin, normalMax) {
  if (value < normalMin * 0.9 || value > normalMax * 1.1) return 'critical';
  if (value < normalMin || value > normalMax) return 'warning';
  return 'normal';
}
```

#### Backend Architecture

**Service**: Health Monitoring Service

**Tech Stack**:
- **Backend**: Python + FastAPI (for AI integration)
- **Database**:
  - PostgreSQL (structured health records)
  - InfluxDB (time-series vital signs data)
  - Redis (real-time cache)
- **Message Queue**: MQTT for IoT devices, Redis Pub/Sub for internal
- **Real-time**: Socket.io / WebSocket server
- **AI/ML**: TensorFlow for anomaly detection

**API Endpoints**:
```python
# Get latest vital signs
GET /api/health/:elderlyId/vitals/latest
Response: {
  "elderlyId": 1,
  "timestamp": "2025-11-25T14:30:00Z",
  "heartRate": 72,
  "bloodPressure": {
    "systolic": 120,
    "diastolic": 80
  },
  "spo2": 98,
  "temperature": 36.8,
  "source": "smartwatch",
  "deviceId": "SW-001",
  "status": "normal",
  "alerts": []
}

# Get vital signs history
GET /api/health/:elderlyId/vitals/history?hours=24&metric=heartRate
Response: {
  "metric": "heartRate",
  "unit": "bpm",
  "dataPoints": [
    { "timestamp": "2025-11-25T14:00:00Z", "value": 72 },
    { "timestamp": "2025-11-25T14:05:00Z", "value": 74 },
    ...
  ],
  "statistics": {
    "min": 65,
    "max": 82,
    "avg": 72,
    "stdDev": 4.2
  }
}

# Record vital signs (from IoT devices)
POST /api/health/:elderlyId/vitals
Authorization: Bearer <device_token>
Body: {
  "heartRate": 72,
  "bloodPressure": { "systolic": 120, "diastolic": 80 },
  "spo2": 98,
  "temperature": 36.8,
  "deviceId": "SW-001",
  "timestamp": "2025-11-25T14:30:00Z"
}
Response: {
  "recorded": true,
  "status": "normal",
  "alerts": []
}
```

**Health Monitoring Service Implementation**:
```python
# health_service.py
from fastapi import FastAPI, WebSocket
from influxdb_client import InfluxDBClient
import asyncio
import json

app = FastAPI()

class HealthMonitoringService:
    def __init__(self):
        self.influx = InfluxDBClient(url="http://localhost:8086", token="...")
        self.redis = redis.Redis(host='localhost', port=6379)
        self.mqtt_client = mqtt.Client()
        self.websocket_connections = {}

    async def process_vital_signs(self, elderly_id: int, data: dict):
        """Process incoming vital signs from IoT devices"""

        # 1. Validate data
        validated_data = self.validate_vitals(data)

        # 2. Store in InfluxDB (time-series)
        await self.store_time_series(elderly_id, validated_data)

        # 3. Cache latest in Redis
        self.redis.setex(
            f"vitals:latest:{elderly_id}",
            300,  # 5-minute TTL
            json.dumps(validated_data)
        )

        # 4. Check for anomalies using AI
        alerts = await self.check_anomalies(elderly_id, validated_data)

        # 5. Broadcast to connected dashboards via WebSocket
        await self.broadcast_update(elderly_id, {
            **validated_data,
            "alerts": alerts
        })

        # 6. Trigger alerts if critical
        if alerts:
            await self.trigger_alerts(elderly_id, alerts)

        return validated_data

    def validate_vitals(self, data: dict) -> dict:
        """Validate vital signs are within possible human ranges"""
        rules = {
            "heartRate": (20, 200),
            "bloodPressure.systolic": (50, 250),
            "bloodPressure.diastolic": (30, 150),
            "spo2": (70, 100),
            "temperature": (30.0, 42.0)
        }

        for key, (min_val, max_val) in rules.items():
            value = self.get_nested(data, key)
            if value < min_val or value > max_val:
                raise ValueError(f"{key} out of valid range: {value}")

        return data

    async def check_anomalies(self, elderly_id: int, data: dict) -> list:
        """Use AI model to detect anomalies"""

        # Get baseline for this elderly
        baseline = await self.get_baseline(elderly_id)

        # Load AI model
        model = load_model('health_anomaly_detector.h5')

        # Prepare features
        features = [
            data['heartRate'],
            data['bloodPressure']['systolic'],
            data['bloodPressure']['diastolic'],
            data['spo2'],
            data['temperature']
        ]

        # Predict anomaly
        prediction = model.predict([features])
        is_anomaly = prediction[0] > 0.8  # 80% threshold

        alerts = []

        # Rule-based checks
        if data['heartRate'] > 120 or data['heartRate'] < 50:
            alerts.append({
                "type": "heart_rate_abnormal",
                "severity": "critical",
                "message": f"Heart rate {data['heartRate']} outside normal range"
            })

        if data['bloodPressure']['systolic'] > 180:
            alerts.append({
                "type": "hypertension_crisis",
                "severity": "critical",
                "message": "Blood pressure critically high"
            })

        if data['spo2'] < 90:
            alerts.append({
                "type": "low_oxygen",
                "severity": "critical",
                "message": "Oxygen saturation critically low"
            })

        if is_anomaly:
            alerts.append({
                "type": "ai_anomaly_detected",
                "severity": "warning",
                "message": "AI detected unusual vital signs pattern"
            })

        return alerts

    async def store_time_series(self, elderly_id: int, data: dict):
        """Store vital signs in InfluxDB for time-series analysis"""

        write_api = self.influx.write_api()

        points = [
            Point("vital_signs")
                .tag("elderly_id", str(elderly_id))
                .tag("device_id", data['deviceId'])
                .field("heart_rate", data['heartRate'])
                .field("bp_systolic", data['bloodPressure']['systolic'])
                .field("bp_diastolic", data['bloodPressure']['diastolic'])
                .field("spo2", data['spo2'])
                .field("temperature", data['temperature'])
                .time(data['timestamp'])
        ]

        write_api.write(bucket="elderly_care", record=points)

# WebSocket endpoint for real-time updates
@app.websocket("/ws/vitals/{elderly_id}")
async def websocket_vitals(websocket: WebSocket, elderly_id: int):
    await websocket.accept()
    service.websocket_connections[elderly_id] = websocket

    try:
        # Send initial data
        latest = await service.get_latest_vitals(elderly_id)
        await websocket.send_json(latest)

        # Keep connection alive and wait for updates
        while True:
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        del service.websocket_connections[elderly_id]
```

**MQTT Integration (IoT Devices)**:
```python
# mqtt_bridge.py
import paho.mqtt.client as mqtt
import json

def on_message(client, userdata, message):
    """Handle incoming MQTT messages from IoT devices"""

    topic = message.topic
    payload = json.loads(message.payload.decode())

    # Topic format: elderly/{elderly_id}/vitals
    parts = topic.split('/')
    elderly_id = int(parts[1])

    # Process vital signs
    asyncio.run(health_service.process_vital_signs(elderly_id, payload))

# Connect to MQTT broker
client = mqtt.Client()
client.on_message = on_message
client.connect("mqtt.eldercare.com", 1883)
client.subscribe("elderly/+/vitals")
client.loop_forever()
```

**Database Schema**:
```sql
-- PostgreSQL: Structured health records
CREATE TABLE health_records (
  id SERIAL PRIMARY KEY,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  recorded_at TIMESTAMP NOT NULL,
  heart_rate INTEGER,
  bp_systolic INTEGER,
  bp_diastolic INTEGER,
  spo2 INTEGER,
  temperature DECIMAL(4,1),
  device_id VARCHAR(50),
  source VARCHAR(50), -- smartwatch, manual, etc.
  notes TEXT,
  recorded_by INTEGER REFERENCES users(id),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_health_records_elderly_time
  ON health_records(elderly_id, recorded_at DESC);

-- InfluxDB Schema (automatically handled)
-- Bucket: elderly_care
-- Measurement: vital_signs
-- Tags: elderly_id, device_id
-- Fields: heart_rate, bp_systolic, bp_diastolic, spo2, temperature
-- Time: timestamp
```

**Data Flow**:
```
1. Wearable device (smartwatch) measures vitals
2. Device sends via Bluetooth → Mobile app → HTTP POST
   OR
   Device with WiFi → Direct MQTT publish
3. MQTT message: elderly/001/vitals → MQTT Broker
4. Health Service subscribes → Receives message
5. Validate data → Store in InfluxDB (time-series)
6. Cache in Redis (latest values)
7. AI model checks for anomalies
8. If anomaly → Generate alert → Notify service
9. Broadcast via WebSocket to dashboard
10. Dashboard updates UI in real-time
```

**Scope**:
- ✅ Real-time display of 4 vital signs
- ✅ WebSocket updates every 5 seconds
- ✅ Color-coded status indicators
- ⚠️ Historical trend analysis
- ⚠️ AI anomaly detection
- ⚠️ Manual vital entry for nurses
- ⚠️ Multi-device aggregation
- ⚠️ Vital signs correlation analysis
- ⚠️ Export to PDF/Excel for doctors

---

### FEATURE 6: AI Health Prediction

**Frontend Location**: `.ai-prediction`

#### Purpose
- Display AI-powered health stability prediction
- Gives caregivers confidence in elderly's health outlook
- Based on 30-day historical analysis
- Percentage score for easy interpretation

#### Frontend Implementation
```javascript
// Fetch AI prediction
async function fetchHealthPrediction(elderlyId) {
  const response = await fetch(
    `/api/ai/health-prediction/${elderlyId}`,
    {
      headers: { 'Authorization': `Bearer ${token}` }
    }
  );

  const prediction = await response.json();

  // Update UI
  document.querySelector('.ai-prediction span').textContent =
    `AI Prediction: ${prediction.stabilityScore}% Health Stability`;

  // Color based on score
  const color = prediction.stabilityScore >= 80 ? 'green' :
                prediction.stabilityScore >= 60 ? 'orange' : 'red';

  document.querySelector('.ai-prediction').style.borderColor = color;
}
```

#### Backend Architecture

**Service**: AI Prediction Service

**Tech Stack**:
- **Backend**: Python + FastAPI
- **ML Framework**: TensorFlow 2.x / PyTorch
- **Model**: LSTM Neural Network
- **Model Serving**: TensorFlow Serving / FastAPI
- **Database**: PostgreSQL (health history), InfluxDB (time-series)
- **Cache**: Redis (prediction results - 1-hour TTL)

**API Endpoints**:
```python
# Get health stability prediction
GET /api/ai/health-prediction/:elderlyId
Response: {
  "elderlyId": 1,
  "stabilityScore": 95,
  "confidence": 0.92,
  "prediction": "stable",
  "riskFactors": [],
  "recommendations": [
    "Continue current medication regimen",
    "Maintain current activity level"
  ],
  "predictedFor": "next_7_days",
  "basedOnDays": 30,
  "lastUpdated": "2025-11-25T14:00:00Z"
}

# Trigger model retraining (admin)
POST /api/ai/health-prediction/retrain
Body: { "elderlyId": 1, "historical_days": 90 }
Response: {
  "trainingJobId": "TJ-001",
  "status": "queued",
  "estimatedTime": "15 minutes"
}
```

**ML Model Implementation**:
```python
# ml_models/health_prediction.py
import tensorflow as tf
from tensorflow import keras
import numpy as np

class HealthPredictionModel:
    def __init__(self):
        self.model = self.build_model()
        self.scaler = StandardScaler()

    def build_model(self):
        """Build LSTM neural network for health prediction"""

        model = keras.Sequential([
            # Input: 30 days × 8 features
            keras.layers.LSTM(128, return_sequences=True, input_shape=(30, 8)),
            keras.layers.Dropout(0.3),
            keras.layers.LSTM(64, return_sequences=False),
            keras.layers.Dropout(0.3),
            keras.layers.Dense(32, activation='relu'),
            keras.layers.BatchNormalization(),
            keras.layers.Dense(16, activation='relu'),
            keras.layers.Dense(1, activation='sigmoid')  # 0-1 stability score
        ])

        model.compile(
            optimizer='adam',
            loss='binary_crossentropy',
            metrics=['accuracy', 'AUC']
        )

        return model

    def prepare_features(self, elderly_id: int, days: int = 30):
        """Prepare input features from historical data"""

        # Fetch last 30 days of data
        data = fetch_health_history(elderly_id, days)

        # Features: [heart_rate, bp_sys, bp_dia, spo2, temp, steps, sleep, med_adherence]
        features = []
        for day in data:
            features.append([
                day['heart_rate_avg'],
                day['bp_systolic_avg'],
                day['bp_diastolic_avg'],
                day['spo2_avg'],
                day['temperature_avg'],
                day['steps'],
                day['sleep_hours'],
                day['medication_adherence']  # 0-1
            ])

        # Normalize features
        features_scaled = self.scaler.fit_transform(features)

        # Reshape for LSTM: (samples, timesteps, features)
        return features_scaled.reshape(1, 30, 8)

    async def predict_health_stability(self, elderly_id: int):
        """Generate health stability prediction"""

        # Check cache first
        cached = redis.get(f"prediction:{elderly_id}")
        if cached:
            return json.loads(cached)

        # Prepare input data
        features = self.prepare_features(elderly_id)

        # Predict
        prediction = self.model.predict(features)[0][0]
        stability_score = int(prediction * 100)

        # Analyze risk factors
        risk_factors = self.analyze_risk_factors(elderly_id)

        # Generate recommendations
        recommendations = self.generate_recommendations(
            stability_score,
            risk_factors
        )

        result = {
            "elderlyId": elderly_id,
            "stabilityScore": stability_score,
            "confidence": float(prediction),
            "prediction": self.categorize_stability(stability_score),
            "riskFactors": risk_factors,
            "recommendations": recommendations,
            "predictedFor": "next_7_days",
            "basedOnDays": 30,
            "lastUpdated": datetime.now().isoformat()
        }

        # Cache for 1 hour
        redis.setex(
            f"prediction:{elderly_id}",
            3600,
            json.dumps(result)
        )

        return result

    def categorize_stability(self, score: int) -> str:
        """Categorize stability score"""
        if score >= 80:
            return "stable"
        elif score >= 60:
            return "moderate_risk"
        else:
            return "high_risk"

    def analyze_risk_factors(self, elderly_id: int) -> list:
        """Identify risk factors from recent data"""

        recent_data = fetch_recent_health_data(elderly_id, days=7)
        risks = []

        # Check for declining trends
        if recent_data['heart_rate_trend'] == 'increasing':
            risks.append({
                "factor": "increasing_heart_rate",
                "severity": "medium",
                "description": "Heart rate has been trending up over past week"
            })

        if recent_data['activity_trend'] == 'decreasing':
            risks.append({
                "factor": "decreased_activity",
                "severity": "low",
                "description": "Physical activity has decreased by 15%"
            })

        if recent_data['medication_adherence'] < 0.85:
            risks.append({
                "factor": "poor_medication_adherence",
                "severity": "high",
                "description": "Medication adherence below 85%"
            })

        return risks

    def generate_recommendations(self, score: int, risks: list) -> list:
        """Generate personalized recommendations"""

        recommendations = []

        if score >= 80:
            recommendations.append("Continue current care regimen")
            recommendations.append("Maintain regular activity levels")
        else:
            recommendations.append("Schedule check-up with doctor")

            for risk in risks:
                if risk['factor'] == 'decreased_activity':
                    recommendations.append("Increase light physical activity")
                elif risk['factor'] == 'poor_medication_adherence':
                    recommendations.append("Review medication schedule with patient")

        return recommendations

# FastAPI endpoint
@app.get("/api/ai/health-prediction/{elderly_id}")
async def get_health_prediction(elderly_id: int):
    model = HealthPredictionModel()
    prediction = await model.predict_health_stability(elderly_id)
    return prediction
```

**Model Training Pipeline**:
```python
# train_model.py
def train_health_prediction_model():
    """Train model on historical data"""

    # Load training data from database
    # Format: 10,000+ patient-days with outcomes
    data = load_training_data()

    # Features: 30 days × 8 metrics
    X = data[['heart_rate', 'bp_sys', 'bp_dia', 'spo2',
              'temp', 'steps', 'sleep', 'med_adherence']]

    # Label: Health stable (1) or not (0) in next 7 days
    y = data['stable_next_7_days']

    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    # Build and train model
    model = build_model()

    history = model.fit(
        X_train, y_train,
        validation_data=(X_test, y_test),
        epochs=50,
        batch_size=32,
        callbacks=[
            EarlyStopping(patience=5),
            ModelCheckpoint('best_model.h5')
        ]
    )

    # Evaluate
    loss, accuracy, auc = model.evaluate(X_test, y_test)
    print(f"Test Accuracy: {accuracy:.2%}")
    print(f"Test AUC: {auc:.2f}")

    # Save model
    model.save('health_prediction_model.h5')

    # Deploy to production
    deploy_model_to_serving('health_prediction_model.h5')
```

**Database Schema**:
```sql
-- Store prediction history
CREATE TABLE health_predictions (
  id SERIAL PRIMARY KEY,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  prediction_date DATE NOT NULL,
  stability_score INTEGER NOT NULL CHECK (stability_score BETWEEN 0 AND 100),
  confidence DECIMAL(4,3),
  prediction_category VARCHAR(50), -- stable, moderate_risk, high_risk
  risk_factors JSONB,
  recommendations TEXT[],
  model_version VARCHAR(50),
  based_on_days INTEGER DEFAULT 30,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_predictions_elderly_date
  ON health_predictions(elderly_id, prediction_date DESC);

-- Track model performance
CREATE TABLE prediction_outcomes (
  id SERIAL PRIMARY KEY,
  prediction_id INTEGER REFERENCES health_predictions(id),
  actual_outcome VARCHAR(50), -- stable, unstable, hospitalized
  outcome_date DATE NOT NULL,
  was_accurate BOOLEAN,
  notes TEXT,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Scope**:
- ✅ AI-powered 7-day health prediction
- ✅ Based on 30-day historical analysis
- ✅ Percentage score display
- ⚠️ Risk factor identification
- ⚠️ Personalized recommendations
- ⚠️ Prediction confidence interval
- ⚠️ Model retraining pipeline
- ⚠️ Prediction accuracy tracking
- ⚠️ Explainable AI (why this prediction)

---

### FEATURE 7: Robot Caregiver Status & Control

**Frontend Location**: `.status-card.robot-status`

#### Purpose
- Monitor robot caregiver status in real-time
- Display battery level, location, current task
- Quick action buttons for common commands
- One-click access to full robot control interface

#### Frontend Implementation
```javascript
// Subscribe to robot status updates
socket.on('robot_status_update', (data) => {
  document.getElementById('robotBattery').textContent = `${data.battery}%`;
  document.querySelector('.robot-details p:nth-child(3)').innerHTML =
    `<strong>Location:</strong> ${data.location}`;
  document.querySelector('.robot-details p:nth-child(4)').innerHTML =
    `<strong>Current Task:</strong> ${data.currentTask}`;

  // Update battery color
  const batteryElement = document.getElementById('robotBattery');
  batteryElement.style.color = data.battery < 20 ? 'red' :
                                data.battery < 50 ? 'orange' : 'green';
});

// Quick action buttons
document.querySelector('.action-btn:nth-child(1)').addEventListener('click', () => {
  sendRobotCommand('start_conversation');
});

document.querySelector('.action-btn:nth-child(2)').addEventListener('click', () => {
  sendRobotCommand('assist_mode');
});

document.querySelector('.action-btn:nth-child(3)').addEventListener('click', () => {
  initiateVideoCall();
});

async function sendRobotCommand(action) {
  const response = await fetch('/api/robot/command', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${token}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      elderlyId: currentElderlyId,
      action: action
    })
  });

  const result = await response.json();
  showNotification(`Robot ${action} initiated`, 'success');
}
```

#### Backend Architecture

**Service**: Robot Control Service

**Tech Stack**:
- **Backend**: Python + FastAPI (for ROS2 integration)
- **Robot OS**: ROS2 Humble
- **Communication**: MQTT + WebSocket + ROS2 Topics
- **Database**: PostgreSQL (robot logs), Redis (current state)
- **Video Streaming**: WebRTC / Jitsi
- **AI**: OpenAI GPT-4 for conversation

**API Endpoints**:
```python
# Get robot status
GET /api/robot/:elderlyId/status
Response: {
  "robotId": "ROBOT-001",
  "elderlyId": 1,
  "status": "active" | "idle" | "charging" | "offline",
  "battery": 87,
  "location": {
    "room": "Living Room",
    "coordinates": { "x": 2.5, "y": 3.1, "theta": 1.57 }
  },
  "currentTask": "companion_mode",
  "mode": "companion" | "assistant" | "security" | "emergency",
  "capabilities": ["conversation", "navigation", "fetch", "video_call"],
  "sensors": {
    "lidar": "operational",
    "camera": "operational",
    "microphone": "operational",
    "speaker": "operational"
  },
  "lastSeen": "2025-11-25T14:30:00Z"
}

# Send command to robot
POST /api/robot/:elderlyId/command
Body: {
  "action": "move_forward" | "turn_left" | "start_conversation" |
            "fetch_item" | "patrol" | "go_to_charging",
  "parameters": {...}
}
Response: {
  "commandId": "CMD-001",
  "status": "accepted" | "executing" | "completed" | "failed",
  "message": "Command sent to robot"
}

# Get robot command history
GET /api/robot/:elderlyId/commands/history?limit=50
Response: {
  "commands": [
    {
      "commandId": "CMD-001",
      "action": "move_forward",
      "issuedBy": "caregiver_123",
      "timestamp": "2025-11-25T14:30:00Z",
      "status": "completed",
      "duration": 5.2
    },
    ...
  ]
}

# Start video call via robot
POST /api/robot/:elderlyId/video-call/start
Body: {
  "callerId": "user_123",
  "callerName": "Siti Tan"
}
Response: {
  "callId": "CALL-001",
  "roomUrl": "https://meet.eldercare.com/room-001",
  "status": "ringing"
}
```

**Robot Control Service Implementation**:
```python
# robot_service.py
from fastapi import FastAPI, WebSocket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
import paho.mqtt.client as mqtt

app = FastAPI()

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')

        # ROS2 publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ROS2 subscribers
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # MQTT for cloud communication
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("mqtt.eldercare.com", 1883)
        self.mqtt_client.on_message = self.mqtt_callback
        self.mqtt_client.subscribe("robot/+/commands")

        # State
        self.robot_state = {
            "battery": 100,
            "location": "Living Room",
            "task": "idle",
            "mode": "companion"
        }

    def battery_callback(self, msg):
        """Handle battery updates from robot"""
        battery_percent = int(msg.percentage * 100)
        self.robot_state['battery'] = battery_percent

        # Publish to dashboard via WebSocket
        self.broadcast_status_update()

        # Check if low battery
        if battery_percent < 20:
            self.trigger_low_battery_alert()
            self.autonomous_return_to_charger()

    def mqtt_callback(self, client, userdata, msg):
        """Handle commands from cloud"""
        command = json.loads(msg.payload.decode())
        self.execute_command(command)

    async def execute_command(self, command: dict):
        """Execute robot command"""
        action = command['action']

        if action == 'move_forward':
            await self.move_forward()
        elif action == 'turn_left':
            await self.turn_left()
        elif action == 'start_conversation':
            await self.start_conversation()
        elif action == 'fetch_item':
            await self.fetch_item(command['parameters']['item'])
        elif action == 'patrol':
            await self.start_patrol_mode()
        elif action == 'go_to_charging':
            await self.navigate_to_charger()

        # Log command execution
        await self.log_command(command)

    async def move_forward(self, distance: float = 0.5):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.2  # m/s

        # Publish for 2.5 seconds (0.5m at 0.2 m/s)
        duration = distance / 0.2
        for _ in range(int(duration * 10)):
            self.cmd_vel_pub.publish(twist)
            await asyncio.sleep(0.1)

        # Stop
        self.cmd_vel_pub.publish(Twist())

    async def start_conversation(self):
        """Initiate AI conversation with elderly"""

        # Update status
        self.robot_state['task'] = "conversing"
        self.broadcast_status_update()

        # Play greeting audio
        await self.play_audio("Hello! How are you feeling today?")

        # Start listening for response
        await self.listen_and_respond()

    async def listen_and_respond(self):
        """Listen to elderly and respond using GPT-4"""

        while True:
            # Capture audio from microphone
            audio = await self.capture_audio(duration=5)

            # Speech-to-text
            text = await self.speech_to_text(audio)

            if not text or text == "":
                break

            # Generate response using GPT-4
            response = await self.generate_response(text)

            # Text-to-speech
            await self.play_audio(response)

    async def generate_response(self, user_message: str) -> str:
        """Generate conversational response using GPT-4"""

        # Call OpenAI API
        response = await openai.ChatCompletion.acreate(
            model="gpt-4",
            messages=[
                {
                    "role": "system",
                    "content": "You are a compassionate caregiver robot talking to an elderly person. Be warm, patient, and supportive."
                },
                {
                    "role": "user",
                    "content": user_message
                }
            ],
            max_tokens=150,
            temperature=0.7
        )

        return response.choices[0].message.content

    async def fetch_item(self, item_name: str):
        """Navigate to item and bring it to elderly"""

        # Update status
        self.robot_state['task'] = f"fetching_{item_name}"
        self.broadcast_status_update()

        # Use object detection to locate item
        item_location = await self.detect_object(item_name)

        if not item_location:
            await self.play_audio(f"Sorry, I couldn't find the {item_name}")
            return

        # Navigate to item
        await self.navigate_to(item_location)

        # Use gripper to pick up item
        await self.pick_up_object()

        # Navigate back to elderly
        await self.navigate_to_elderly()

        # Release item
        await self.release_object()

        await self.play_audio(f"Here is your {item_name}")

    def broadcast_status_update(self):
        """Broadcast robot status to all connected dashboards"""
        message = {
            "type": "robot_status_update",
            "data": self.robot_state
        }

        # Publish via WebSocket to all connected clients
        for client in websocket_connections.values():
            asyncio.create_task(client.send_json(message))

# FastAPI endpoints
@app.post("/api/robot/{elderly_id}/command")
async def send_robot_command(elderly_id: int, command: dict):
    # Publish command to robot via MQTT
    mqtt_client.publish(
        f"robot/{elderly_id}/commands",
        json.dumps(command)
    )

    return {
        "commandId": generate_command_id(),
        "status": "accepted",
        "message": "Command sent to robot"
    }

@app.websocket("/ws/robot/{elderly_id}")
async def websocket_robot_status(websocket: WebSocket, elderly_id: int):
    await websocket.accept()
    websocket_connections[elderly_id] = websocket

    try:
        while True:
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        del websocket_connections[elderly_id]
```

**ROS2 Integration**:
```python
# ROS2 Launch File
# ros2_ws/src/elderly_care_robot/launch/robot.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot control node
        Node(
            package='elderly_care_robot',
            executable='robot_control_node',
            name='robot_control',
            parameters=[{
                'max_speed': 0.3,
                'safety_distance': 0.5
            }]
        ),

        # Navigation node
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='navigation'
        ),

        # LIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame'
            }]
        ),

        # Camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30
            }]
        ),

        # MQTT bridge node
        Node(
            package='elderly_care_robot',
            executable='mqtt_bridge_node',
            name='mqtt_bridge'
        )
    ])
```

**Database Schema**:
```sql
CREATE TABLE robot_status (
  id SERIAL PRIMARY KEY,
  robot_id VARCHAR(50) NOT NULL,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  status VARCHAR(50), -- active, idle, charging, offline
  battery_level INTEGER CHECK (battery_level BETWEEN 0 AND 100),
  location_room VARCHAR(100),
  location_x DECIMAL(10,2),
  location_y DECIMAL(10,2),
  location_theta DECIMAL(10,2),
  current_task VARCHAR(100),
  mode VARCHAR(50), -- companion, assistant, security, emergency
  timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE robot_commands (
  id SERIAL PRIMARY KEY,
  command_id VARCHAR(50) UNIQUE NOT NULL,
  robot_id VARCHAR(50) NOT NULL,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  action VARCHAR(100) NOT NULL,
  parameters JSONB,
  issued_by INTEGER REFERENCES users(id),
  issued_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  status VARCHAR(50), -- queued, executing, completed, failed
  started_at TIMESTAMP,
  completed_at TIMESTAMP,
  duration_seconds DECIMAL(10,2),
  error_message TEXT
);

CREATE TABLE robot_tasks (
  id SERIAL PRIMARY KEY,
  robot_id VARCHAR(50) NOT NULL,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  task_type VARCHAR(100), -- conversation, fetch_item, patrol, etc.
  task_details JSONB,
  started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  completed_at TIMESTAMP,
  success BOOLEAN,
  notes TEXT
);

-- Indexes
CREATE INDEX idx_robot_status_latest
  ON robot_status(robot_id, timestamp DESC);
CREATE INDEX idx_robot_commands_status
  ON robot_commands(status, issued_at DESC);
```

**Data Flow**:
```
1. Dashboard sends command → POST /api/robot/command
2. API Gateway → Robot Control Service
3. Service publishes to MQTT: robot/001/commands
4. Robot's Raspberry Pi subscribes → Receives command
5. ROS2 node processes command → Executes action
6. Robot publishes status updates → MQTT: robot/001/status
7. Service receives status → Broadcasts via WebSocket
8. Dashboard updates UI in real-time
```

**Scope**:
- ✅ Real-time robot status monitoring
- ✅ Battery level display with alerts
- ✅ Current location and task display
- ✅ Quick action buttons (chat, assist, video call)
- ⚠️ Autonomous navigation
- ⚠️ Object detection and fetching
- ⚠️ AI conversation with GPT-4
- ⚠️ Video calling integration
- ⚠️ Patrol mode with anomaly detection
- ⚠️ Emergency response mode
- ⚠️ Medication delivery
- ⚠️ Voice command recognition

---

## [CONTINUED IN NEXT PART DUE TO LENGTH]

This document is getting very long. Shall I continue with the remaining features, or would you like me to:

1. **Continue with all features** (will create multiple files)
2. **Focus on specific features** you're most interested in
3. **Create a separate document** for each major section

The remaining features to document:
- Activity & Safety Monitoring
- IoT Smart Home Control (6 devices)
- Health Trends Charts
- Medication Schedule
- AI Insights & Predictions (4 cards)
- Emergency Contacts
- Real-time Activity Feed

Please let me know how you'd like to proceed!
