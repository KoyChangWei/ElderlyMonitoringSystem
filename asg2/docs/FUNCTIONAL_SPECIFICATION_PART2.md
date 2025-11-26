# Elderly Care Monitoring Dashboard
# Functional Specification - Part 2 (Continued)

**Document Version**: 1.0
**Date**: November 25, 2025

---

## Continued from Part 1...

### FEATURE 8: Activity & Safety Monitoring

**Frontend Location**: `.status-card.activity-status`

#### Purpose
- Monitor daily activity levels (steps, sleep)
- Fall detection status
- Mood/emotional state tracking
- Quick access to safety features

#### Frontend Implementation
```javascript
// Real-time activity updates
socket.on('activity_update', (data) => {
  document.querySelector('.activity-value:nth-of-type(1)').textContent =
    `${data.steps} steps`;

  document.querySelector('.activity-value:nth-of-type(2)').textContent =
    `${data.sleepHours} hrs (${data.sleepQuality})`;

  document.querySelector('.activity-value:nth-of-type(3)').textContent =
    data.fallsToday > 0 ? `${data.fallsToday} Falls Detected` : 'No Falls Detected';

  document.querySelector('.activity-value:nth-of-type(4)').textContent =
    `${data.moodScore}% (${data.moodLabel})`;
});
```

#### Backend Architecture

**Service**: Activity Monitoring Service

**Tech Stack**:
- **Backend**: Node.js + Express.js
- **Database**: InfluxDB (time-series steps), PostgreSQL (daily summaries)
- **AI/ML**: Python + TensorFlow (mood analysis from facial recognition)
- **IoT**: Smart shoes, smartwatch, cameras for fall detection

**API Endpoints**:
```javascript
// Get today's activity summary
GET /api/activity/:elderlyId/today
Response: {
  "elderlyId": 1,
  "date": "2025-11-25",
  "steps": {
    "current": 2847,
    "goal": 5000,
    "percentage": 57
  },
  "sleep": {
    "hours": 7.5,
    "quality": "Good", // Excellent, Good, Fair, Poor
    "deepSleepMinutes": 120,
    "remSleepMinutes": 90
  },
  "falls": {
    "count": 0,
    "incidents": []
  },
  "mood": {
    "score": 85,
    "label": "Happy", // Happy, Neutral, Sad, Anxious
    "confidence": 0.92
  },
  "activityLevel": "moderate", // low, moderate, high
  "lastUpdated": "2025-11-25T14:30:00Z"
}

// Get activity history
GET /api/activity/:elderlyId/history?days=7
Response: {
  "data": [
    {
      "date": "2025-11-25",
      "steps": 2847,
      "sleepHours": 7.5,
      "falls": 0,
      "moodScore": 85
    },
    ...
  ],
  "trends": {
    "stepsChange": -15, // percentage
    "sleepChange": +5,
    "moodChange": -3
  }
}

// Record fall incident
POST /api/activity/:elderlyId/fall-incident
Body: {
  "timestamp": "2025-11-25T14:30:00Z",
  "location": "Bathroom",
  "severity": "minor" | "moderate" | "severe",
  "detected_by": "ai_camera" | "wearable" | "manual",
  "confidence": 0.95,
  "videoClipId": "CLIP-001"
}
Response: {
  "incidentId": "FALL-001",
  "alertSent": true,
  "actions": ["notify_caregiver", "check_vitals"]
}
```

**Activity Monitoring Implementation**:
```javascript
// activity_service.js
const InfluxDB = require('influx');
const mqtt = require('mqtt');

class ActivityMonitoringService {
  constructor() {
    this.influx = new InfluxDB.InfluxDB({
      host: 'localhost',
      database: 'elderly_activity',
      schema: [
        {
          measurement: 'steps',
          fields: { count: InfluxDB.FieldType.INTEGER },
          tags: ['elderly_id', 'device_id']
        },
        {
          measurement: 'sleep',
          fields: {
            sleep_stage: InfluxDB.FieldType.STRING,
            heart_rate: InfluxDB.FieldType.INTEGER
          },
          tags: ['elderly_id']
        }
      ]
    });

    this.mqttClient = mqtt.connect('mqtt://localhost:1883');
    this.mqttClient.on('message', this.handleDeviceData.bind(this));
    this.mqttClient.subscribe('elderly/+/activity/#');
  }

  async handleDeviceData(topic, message) {
    const parts = topic.split('/');
    const elderlyId = parseInt(parts[1]);
    const dataType = parts[3]; // steps, sleep, fall

    const data = JSON.parse(message.toString());

    if (dataType === 'steps') {
      await this.processSteps(elderlyId, data);
    } else if (dataType === 'sleep') {
      await this.processSleep(elderlyId, data);
    } else if (dataType === 'fall') {
      await this.processFallIncident(elderlyId, data);
    }
  }

  async processSteps(elderlyId, data) {
    // Store in InfluxDB
    await this.influx.writePoints([
      {
        measurement: 'steps',
        tags: {
          elderly_id: elderlyId.toString(),
          device_id: data.deviceId
        },
        fields: { count: data.steps },
        timestamp: new Date(data.timestamp)
      }
    ]);

    // Get today's total
    const todayTotal = await this.getTodaySteps(elderlyId);

    // Check if goal reached
    const goal = await this.getStepGoal(elderlyId);
    if (todayTotal >= goal && !this.goalReachedToday[elderlyId]) {
      this.goalReachedToday[elderlyId] = true;
      await this.sendCongratulationMessage(elderlyId);
    }

    // Broadcast update
    this.broadcastActivityUpdate(elderlyId, { steps: todayTotal });
  }

  async processSleep(elderlyId, data) {
    // Store sleep data
    await db.query(`
      INSERT INTO sleep_records (elderly_id, date, total_hours, deep_sleep_minutes,
                                rem_sleep_minutes, quality, device_id)
      VALUES ($1, $2, $3, $4, $5, $6, $7)
    `, [
      elderlyId,
      data.date,
      data.totalHours,
      data.deepSleepMinutes,
      data.remSleepMinutes,
      this.calculateSleepQuality(data),
      data.deviceId
    ]);

    // Analyze sleep patterns
    const analysis = await this.analyzeSleepPattern(elderlyId);

    if (analysis.poorSleepStreak >= 3) {
      await this.createAlert({
        elderlyId,
        type: 'poor_sleep_pattern',
        severity: 'warning',
        message: 'Sleep quality has been poor for 3+ nights'
      });
    }
  }

  async processFallIncident(elderlyId, data) {
    // Log fall incident
    const incidentId = await db.query(`
      INSERT INTO fall_incidents (elderly_id, timestamp, location, severity,
                                  detected_by, confidence, video_clip_id)
      VALUES ($1, $2, $3, $4, $5, $6, $7)
      RETURNING id
    `, [
      elderlyId,
      data.timestamp,
      data.location,
      data.severity,
      data.detectedBy,
      data.confidence,
      data.videoClipId
    ]);

    // Immediate actions
    await Promise.all([
      this.sendEmergencyAlert(elderlyId, 'fall_detected'),
      this.checkVitalSigns(elderlyId),
      this.activateRobotResponse(elderlyId),
      this.notifyFamilyMembers(elderlyId, 'Fall detected')
    ]);

    // Broadcast to dashboard
    this.broadcastActivityUpdate(elderlyId, {
      fallDetected: true,
      incidentId: incidentId.rows[0].id
    });

    return incidentId.rows[0].id;
  }

  calculateSleepQuality(sleepData) {
    // Calculate quality based on duration and sleep stages
    const { totalHours, deepSleepMinutes, remSleepMinutes } = sleepData;

    let score = 0;

    // Duration (0-40 points)
    if (totalHours >= 7 && totalHours <= 9) {
      score += 40;
    } else if (totalHours >= 6 && totalHours <= 10) {
      score += 30;
    } else {
      score += 20;
    }

    // Deep sleep (0-30 points)
    if (deepSleepMinutes >= 90) {
      score += 30;
    } else if (deepSleepMinutes >= 60) {
      score += 20;
    } else {
      score += 10;
    }

    // REM sleep (0-30 points)
    if (remSleepMinutes >= 90) {
      score += 30;
    } else if (remSleepMinutes >= 60) {
      score += 20;
    } else {
      score += 10;
    }

    // Convert to quality label
    if (score >= 85) return 'Excellent';
    if (score >= 70) return 'Good';
    if (score >= 50) return 'Fair';
    return 'Poor';
  }

  async analyzeMood(elderlyId) {
    // Use facial recognition from camera feeds
    // Analyze facial expressions using AI model
    const recentImages = await this.getRecentFaceImages(elderlyId, hours=1);

    const emotions = await Promise.all(
      recentImages.map(img => this.detectEmotion(img))
    );

    // Aggregate emotions
    const moodScores = {
      happy: 0,
      neutral: 0,
      sad: 0,
      anxious: 0
    };

    emotions.forEach(emotion => {
      moodScores[emotion.label] += emotion.confidence;
    });

    // Calculate overall mood score (0-100)
    const moodScore = (
      moodScores.happy * 1.0 +
      moodScores.neutral * 0.7 +
      moodScores.sad * 0.3 +
      moodScores.anxious * 0.2
    ) / emotions.length * 100;

    // Determine label
    let label = 'Neutral';
    if (moodScore >= 80) label = 'Happy';
    else if (moodScore >= 60) label = 'Content';
    else if (moodScore >= 40) label = 'Neutral';
    else if (moodScore >= 20) label = 'Sad';
    else label = 'Distressed';

    return {
      score: Math.round(moodScore),
      label: label,
      confidence: this.calculateConfidence(emotions)
    };
  }
}
```

**Database Schema**:
```sql
-- Daily activity summaries
CREATE TABLE daily_activity (
  id SERIAL PRIMARY KEY,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  date DATE NOT NULL,
  steps INTEGER DEFAULT 0,
  steps_goal INTEGER DEFAULT 5000,
  distance_km DECIMAL(10,2),
  active_minutes INTEGER,
  calories_burned INTEGER,
  sleep_hours DECIMAL(3,1),
  sleep_quality VARCHAR(20),
  mood_score INTEGER CHECK (mood_score BETWEEN 0 AND 100),
  mood_label VARCHAR(50),
  falls_count INTEGER DEFAULT 0,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(elderly_id, date)
);

-- Fall incidents
CREATE TABLE fall_incidents (
  id SERIAL PRIMARY KEY,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  timestamp TIMESTAMP NOT NULL,
  location VARCHAR(100),
  severity VARCHAR(20), -- minor, moderate, severe
  detected_by VARCHAR(50), -- ai_camera, wearable, manual
  confidence DECIMAL(4,3),
  video_clip_id VARCHAR(100),
  injury_reported BOOLEAN DEFAULT FALSE,
  medical_attention_required BOOLEAN DEFAULT FALSE,
  response_time_seconds INTEGER,
  notes TEXT,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Sleep records
CREATE TABLE sleep_records (
  id SERIAL PRIMARY KEY,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  date DATE NOT NULL,
  sleep_start TIMESTAMP,
  sleep_end TIMESTAMP,
  total_hours DECIMAL(3,1),
  deep_sleep_minutes INTEGER,
  rem_sleep_minutes INTEGER,
  light_sleep_minutes INTEGER,
  awake_minutes INTEGER,
  quality VARCHAR(20), -- Excellent, Good, Fair, Poor
  device_id VARCHAR(50),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(elderly_id, date)
);

-- Mood tracking
CREATE TABLE mood_records (
  id SERIAL PRIMARY KEY,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  timestamp TIMESTAMP NOT NULL,
  mood_score INTEGER CHECK (mood_score BETWEEN 0 AND 100),
  mood_label VARCHAR(50), -- Happy, Content, Neutral, Sad, Anxious, Distressed
  confidence DECIMAL(4,3),
  detection_method VARCHAR(50), -- facial_recognition, voice_analysis, manual
  context TEXT, -- What was happening
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes
CREATE INDEX idx_daily_activity_elderly_date ON daily_activity(elderly_id, date DESC);
CREATE INDEX idx_fall_incidents_elderly_time ON fall_incidents(elderly_id, timestamp DESC);
CREATE INDEX idx_sleep_records_elderly_date ON sleep_records(elderly_id, date DESC);
CREATE INDEX idx_mood_records_elderly_time ON mood_records(elderly_id, timestamp DESC);
```

**Scope**:
- ✅ Real-time step counting
- ✅ Sleep quality tracking
- ✅ Fall detection with alerts
- ✅ Mood/emotion analysis
- ⚠️ Activity pattern analysis
- ⚠️ Gait analysis for fall prevention
- ⚠️ Social interaction tracking
- ⚠️ Cognitive activity monitoring

---

### FEATURE 9-14: IoT Smart Home Control (6 Devices)

**Frontend Location**: `.iot-grid` (6 devices)

#### Purpose
- Control smart home devices remotely
- Monitor device status in real-time
- Create automated rules for comfort and safety
- Toggle switches for quick on/off

#### Devices:
1. Smart Lights (Living Room)
2. Air Conditioning (Temperature control)
3. AI Cameras (4 active cameras)
4. Smart Door Lock (Security)
5. Air Quality Monitor (AQI display)
6. Voice Assistant (Always listening)

#### Frontend Implementation
```javascript
async function toggleDevice(checkbox, deviceType) {
  const isOn = checkbox.checked;

  try {
    const response = await fetch('/api/iot/device/control', {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        elderlyId: currentElderlyId,
        deviceType: deviceType,
        action: isOn ? 'on' : 'off',
        timestamp: new Date().toISOString()
      })
    });

    const result = await response.json();

    if (result.success) {
      showNotification(`${deviceType} turned ${isOn ? 'ON' : 'OFF'}`, 'success');
    } else {
      checkbox.checked = !isOn; // Revert on failure
      showNotification('Failed to control device', 'error');
    }
  } catch (error) {
    checkbox.checked = !isOn;
    showNotification('Error connecting to device', 'error');
  }
}

// WebSocket listener for device status updates
socket.on('iot_device_update', (data) => {
  const deviceElement = document.querySelector(`[data-device="${data.deviceId}"]`);
  if (deviceElement) {
    const toggle = deviceElement.querySelector('input[type="checkbox"]');
    toggle.checked = data.status === 'on';

    // Update display text
    if (data.deviceType === 'ac') {
      deviceElement.querySelector('.device-info p').textContent =
        `${data.temperature}°C - ${data.mode} Mode`;
    } else if (data.deviceType === 'air_quality') {
      deviceElement.querySelector('.device-info p').textContent =
        `${data.aqi} (${data.qualityLabel})`;
    }
  }
});
```

#### Backend Architecture

**Service**: IoT Device Management Service

**Tech Stack**:
- **Backend**: Node.js + Express.js
- **IoT Protocol**: MQTT (primary), Zigbee, WiFi
- **MQTT Broker**: Mosquitto / HiveMQ / AWS IoT Core
- **Database**: PostgreSQL (device registry), Redis (current state)
- **Device Management**: AWS IoT Device Management
- **Automation Engine**: Node-RED / AWS IoT Events

**API Endpoints**:
```javascript
// Get all devices for elderly
GET /api/iot/:elderlyId/devices
Response: {
  "devices": [
    {
      "deviceId": "LIGHT-LR-001",
      "type": "smart_light",
      "name": "Living Room Lights",
      "location": "Living Room",
      "status": "on",
      "properties": {
        "brightness": 75,
        "colorTemp": 3000
      },
      "lastSeen": "2025-11-25T14:30:00Z",
      "online": true
    },
    {
      "deviceId": "AC-LR-001",
      "type": "air_conditioner",
      "name": "Living Room AC",
      "location": "Living Room",
      "status": "on",
      "properties": {
        "temperature": 24,
        "mode": "auto",
        "fanSpeed": "medium"
      },
      "lastSeen": "2025-11-25T14:29:55Z",
      "online": true
    },
    ...
  ],
  "totalDevices": 15,
  "onlineDevices": 14,
  "offlineDevices": 1
}

// Control device
POST /api/iot/device/control
Body: {
  "elderlyId": 1,
  "deviceId": "LIGHT-LR-001",
  "action": "on" | "off" | "set_property",
  "properties": {
    "brightness": 80,
    "colorTemp": 4000
  }
}
Response: {
  "success": true,
  "deviceId": "LIGHT-LR-001",
  "newStatus": "on",
  "properties": {...}
}

// Get device history
GET /api/iot/device/:deviceId/history?hours=24
Response: {
  "deviceId": "LIGHT-LR-001",
  "events": [
    {
      "timestamp": "2025-11-25T14:00:00Z",
      "action": "turned_on",
      "triggeredBy": "caregiver",
      "properties": {...}
    },
    ...
  ]
}

// Create automation rule
POST /api/iot/:elderlyId/automation/rules
Body: {
  "name": "Bedtime Routine",
  "enabled": true,
  "trigger": {
    "type": "time",
    "time": "21:00"
  },
  "conditions": [],
  "actions": [
    {
      "deviceId": "LIGHT-LR-001",
      "action": "off"
    },
    {
      "deviceId": "DOOR-001",
      "action": "lock"
    },
    {
      "deviceId": "AC-LR-001",
      "action": "set_temperature",
      "value": 23
    }
  ]
}
Response: {
  "ruleId": "RULE-001",
  "status": "active"
}
```

**IoT Service Implementation**:
```javascript
// iot_service.js
const mqtt = require('mqtt');
const AWS = require('aws-sdk');

class IoTDeviceService {
  constructor() {
    // Connect to MQTT broker
    this.mqttClient = mqtt.connect('mqtt://mqtt.eldercare.com:1883', {
      username: 'iot_service',
      password: process.env.MQTT_PASSWORD
    });

    // AWS IoT Core (alternative)
    this.iotData = new AWS.IotData({
      endpoint: 'your-iot-endpoint.amazonaws.com'
    });

    // Subscribe to device topics
    this.mqttClient.on('connect', () => {
      this.mqttClient.subscribe('elderly/+/devices/+/status');
      this.mqttClient.subscribe('elderly/+/devices/+/telemetry');
    });

    // Handle incoming messages
    this.mqttClient.on('message', this.handleDeviceMessage.bind(this));
  }

  async handleDeviceMessage(topic, message) {
    const parts = topic.split('/');
    const elderlyId = parseInt(parts[1]);
    const deviceId = parts[3];
    const messageType = parts[4]; // status or telemetry

    const data = JSON.parse(message.toString());

    if (messageType === 'status') {
      await this.updateDeviceStatus(elderlyId, deviceId, data);
    } else if (messageType === 'telemetry') {
      await this.processTelemetry(elderlyId, deviceId, data);
    }

    // Broadcast to connected dashboards
    this.broadcastDeviceUpdate(elderlyId, {
      deviceId,
      ...data
    });
  }

  async controlDevice(elderlyId, deviceId, action, properties = {}) {
    // Get device info
    const device = await this.getDevice(deviceId);

    if (!device) {
      throw new Error('Device not found');
    }

    if (!device.online) {
      throw new Error('Device is offline');
    }

    // Prepare MQTT message
    const command = {
      action: action,
      properties: properties,
      timestamp: new Date().toISOString(),
      commandId: this.generateCommandId()
    };

    // Publish command to device
    const topic = `elderly/${elderlyId}/devices/${deviceId}/commands`;
    this.mqttClient.publish(topic, JSON.stringify(command));

    // Log command
    await this.logDeviceCommand(elderlyId, deviceId, command);

    // Wait for acknowledgment (with timeout)
    const ack = await this.waitForAcknowledgment(deviceId, command.commandId, 5000);

    if (ack.success) {
      // Update device state in database
      await this.updateDeviceState(deviceId, action, properties);

      return {
        success: true,
        deviceId: deviceId,
        newStatus: ack.newStatus,
        properties: ack.properties
      };
    } else {
      throw new Error('Device did not acknowledge command');
    }
  }

  async updateDeviceStatus(elderlyId, deviceId, data) {
    // Update in Redis (for real-time access)
    await redis.setex(
      `device:${deviceId}:status`,
      300, // 5-minute TTL
      JSON.stringify(data)
    );

    // Update in PostgreSQL
    await db.query(`
      UPDATE iot_devices
      SET
        status = $1,
        properties = $2,
        last_seen = $3,
        online = TRUE,
        updated_at = CURRENT_TIMESTAMP
      WHERE device_id = $4
    `, [data.status, data.properties, new Date(), deviceId]);

    // Check if state change requires alert
    if (data.status === 'error' || data.status === 'malfunction') {
      await this.createDeviceAlert(elderlyId, deviceId, data);
    }
  }

  async processTelemetry(elderlyId, deviceId, data) {
    // Store telemetry in time-series database (InfluxDB)
    await influxDB.writePoints([
      {
        measurement: 'device_telemetry',
        tags: {
          elderly_id: elderlyId.toString(),
          device_id: deviceId,
          device_type: data.type
        },
        fields: data.metrics,
        timestamp: new Date(data.timestamp)
      }
    ]);

    // Process specific device types
    if (data.type === 'air_quality') {
      await this.processAirQuality(elderlyId, data.metrics);
    } else if (data.type === 'camera') {
      await this.processCameraFeed(elderlyId, deviceId, data);
    }
  }

  async processAirQuality(elderlyId, metrics) {
    // Check if air quality is poor
    if (metrics.aqi > 100) {
      await this.createAlert({
        elderlyId,
        type: 'poor_air_quality',
        severity: 'warning',
        message: `Air quality is unhealthy (AQI: ${metrics.aqi})`,
        recommendations: [
          'Close windows',
          'Turn on air purifier',
          'Reduce physical activity'
        ]
      });

      // Automatically turn on air purifier if available
      const purifier = await this.findDevice(elderlyId, 'air_purifier');
      if (purifier) {
        await this.controlDevice(elderlyId, purifier.deviceId, 'on');
      }
    }
  }

  // Automation Rules Engine
  async executeAutomationRules(elderlyId, trigger) {
    // Get active rules for this trigger
    const rules = await db.query(`
      SELECT * FROM automation_rules
      WHERE elderly_id = $1
        AND enabled = TRUE
        AND trigger_type = $2
    `, [elderlyId, trigger.type]);

    for (const rule of rules.rows) {
      // Check conditions
      const conditionsMet = await this.evaluateConditions(rule.conditions);

      if (conditionsMet) {
        // Execute actions
        for (const action of rule.actions) {
          try {
            await this.controlDevice(
              elderlyId,
              action.deviceId,
              action.action,
              action.properties
            );
          } catch (error) {
            console.error(`Failed to execute action for rule ${rule.id}:`, error);
          }
        }

        // Log rule execution
        await this.logRuleExecution(rule.id);
      }
    }
  }
}

// Specific device implementations
class SmartLightController {
  async setBrightness(deviceId, brightness) {
    // brightness: 0-100
    return await iotService.controlDevice(
      elderlyId,
      deviceId,
      'set_property',
      { brightness: Math.max(0, Math.min(100, brightness)) }
    );
  }

  async setColorTemperature(deviceId, kelvin) {
    // kelvin: 2700 (warm) - 6500 (cool)
    return await iotService.controlDevice(
      elderlyId,
      deviceId,
      'set_property',
      { colorTemp: kelvin }
    );
  }

  async turnOnGradually(deviceId, duration = 300) {
    // Gradually turn on lights over duration (seconds)
    for (let brightness = 0; brightness <= 100; brightness += 10) {
      await this.setBrightness(deviceId, brightness);
      await new Promise(resolve => setTimeout(resolve, duration / 10 * 1000));
    }
  }
}

class AirConditioningController {
  async setTemperature(deviceId, temperature) {
    // temperature: 18-30°C
    return await iotService.controlDevice(
      elderlyId,
      deviceId,
      'set_temperature',
      { temperature: Math.max(18, Math.min(30, temperature)) }
    );
  }

  async setMode(deviceId, mode) {
    // mode: auto, cool, fan, dry
    return await iotService.controlDevice(
      elderlyId,
      deviceId,
      'set_mode',
      { mode: mode }
    );
  }
}
```

**Database Schema**:
```sql
-- Device registry
CREATE TABLE iot_devices (
  id SERIAL PRIMARY KEY,
  device_id VARCHAR(100) UNIQUE NOT NULL,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  device_type VARCHAR(50) NOT NULL, -- smart_light, ac, camera, door_lock, etc.
  device_name VARCHAR(255) NOT NULL,
  location VARCHAR(100),
  manufacturer VARCHAR(100),
  model VARCHAR(100),
  firmware_version VARCHAR(50),
  status VARCHAR(50), -- on, off, idle, error
  properties JSONB, -- Device-specific properties
  online BOOLEAN DEFAULT FALSE,
  last_seen TIMESTAMP,
  installed_at TIMESTAMP,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Device commands log
CREATE TABLE device_commands (
  id SERIAL PRIMARY KEY,
  device_id VARCHAR(100) REFERENCES iot_devices(device_id),
  command_id VARCHAR(100) UNIQUE NOT NULL,
  action VARCHAR(100) NOT NULL,
  properties JSONB,
  issued_by INTEGER REFERENCES users(id),
  issued_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  acknowledged_at TIMESTAMP,
  success BOOLEAN,
  error_message TEXT
);

-- Automation rules
CREATE TABLE automation_rules (
  id SERIAL PRIMARY KEY,
  rule_id VARCHAR(100) UNIQUE NOT NULL,
  elderly_id INTEGER REFERENCES elderly_profiles(id),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  enabled BOOLEAN DEFAULT TRUE,
  trigger_type VARCHAR(50), -- time, sensor, event, manual
  trigger_config JSONB,
  conditions JSONB[], -- Array of conditions
  actions JSONB[], -- Array of actions
  last_executed TIMESTAMP,
  execution_count INTEGER DEFAULT 0,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Rule execution log
CREATE TABLE rule_executions (
  id SERIAL PRIMARY KEY,
  rule_id VARCHAR(100) REFERENCES automation_rules(rule_id),
  executed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  trigger_data JSONB,
  actions_executed INTEGER,
  actions_failed INTEGER,
  success BOOLEAN,
  error_message TEXT
);

-- Indexes
CREATE INDEX idx_devices_elderly ON iot_devices(elderly_id);
CREATE INDEX idx_devices_status ON iot_devices(status, online);
CREATE INDEX idx_commands_device ON device_commands(device_id, issued_at DESC);
CREATE INDEX idx_rules_elderly ON automation_rules(elderly_id, enabled);
```

**Scope (Per Device)**:

**1. Smart Lights**:
- ✅ On/Off toggle
- ⚠️ Brightness control (0-100%)
- ⚠️ Color temperature (warm/cool)
- ⚠️ Gradual dimming for sleep
- ⚠️ Motion-activated lighting

**2. Air Conditioning**:
- ✅ On/Off toggle
- ⚠️ Temperature control (18-30°C)
- ⚠️ Mode selection (Auto/Cool/Fan/Dry)
- ⚠️ Fan speed control
- ⚠️ Schedule programming
- ⚠️ Smart temperature adjustment based on occupancy

**3. AI Cameras**:
- ✅ On/Off toggle
- ✅ Live view status (4 cameras)
- ⚠️ Motion detection zones
- ⚠️ Person detection
- ⚠️ Fall detection
- ⚠️ Recording management
- ⚠️ Privacy zones
- ⚠️ Two-way audio

**4. Smart Door Lock**:
- ✅ Lock/Unlock toggle
- ⚠️ Auto-lock scheduling
- ⚠️ Access codes management
- ⚠️ Entry log
- ⚠️ Remote unlock for emergencies
- ⚠️ Geofencing (auto-lock when away)

**5. Air Quality Monitor**:
- ✅ Display AQI (Air Quality Index)
- ✅ Quality label (Good/Moderate/Unhealthy)
- ⚠️ PM2.5, PM10 levels
- ⚠️ VOC detection
- ⚠️ CO2 levels
- ⚠️ Automatic air purifier activation
- ⚠️ Alerts for poor air quality

**6. Voice Assistant**:
- ✅ On/Off toggle
- ✅ Listening status
- ⚠️ Voice command recognition
- ⚠️ Custom commands
- ⚠️ Integration with other devices
- ⚠️ Routine creation
- ⚠️ Conversation with elderly

---

### FEATURE 15: Health Trends Chart

**Frontend Location**: `.chart-container > #healthChart`

#### Purpose
- Visual representation of vital signs over 24 hours
- Switchable between heart rate, BP, SpO2, temperature
- Identify patterns and anomalies
- Historical trend analysis

#### Frontend Implementation
```javascript
// Initialize Chart.js
let healthChart = null;

function initializeHealthChart() {
  const ctx = document.getElementById('healthChart').getContext('2d');

  healthChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: generateTimeLabels(24), // Last 24 hours
      datasets: [{
        label: 'Heart Rate (bpm)',
        data: [], // Populated from API
        borderColor: '#E74C3C',
        backgroundColor: 'rgba(231, 76, 60, 0.1)',
        borderWidth: 3,
        tension: 0.4,
        fill: true,
        pointRadius: 4
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: true,
      plugins: {
        legend: { display: true },
        tooltip: {
          mode: 'index',
          intersect: false
        }
      },
      scales: {
        y: { beginAtZero: false },
        x: { display: true }
      }
    }
  });

  // Load initial data
  updateChart();
}

async function updateChart() {
  const metric = document.getElementById('chartSelector').value;
  const data = await fetchHealthTrendData(currentElderlyId, metric, 24);

  // Update chart
  healthChart.data.datasets[0].label = getMetricLabel(metric);
  healthChart.data.datasets[0].data = data.values;
  healthChart.data.datasets[0].borderColor = getMetricColor(metric);
  healthChart.update();
}

async function fetchHealthTrendData(elderlyId, metric, hours) {
  const response = await fetch(
    `/api/health/${elderlyId}/trends?metric=${metric}&hours=${hours}`,
    { headers: { 'Authorization': `Bearer ${token}` } }
  );
  return await response.json();
}
```

#### Backend Architecture

**Service**: Health Analytics Service

**Tech Stack**:
- **Backend**: Python + FastAPI (for data processing)
- **Database**: InfluxDB (time-series queries)
- **Caching**: Redis (5-minute cache for frequently accessed data)
- **Data Processing**: Pandas, NumPy

**API Endpoints**:
```python
# Get health trend data
GET /api/health/:elderlyId/trends?metric=heartRate&hours=24&interval=1h
Response: {
  "metric": "heartRate",
  "unit": "bpm",
  "interval": "1h",
  "dataPoints": [
    {
      "timestamp": "2025-11-25T00:00:00Z",
      "value": 68,
      "quality": "good" // based on number of samples
    },
    {
      "timestamp": "2025-11-25T01:00:00Z",
      "value": 65,
      "quality": "good"
    },
    ...
  ],
  "statistics": {
    "min": 62,
    "max": 85,
    "mean": 72.3,
    "median": 71,
    "stdDev": 5.2
  },
  "normalRange": {
    "min": 60,
    "max": 100
  },
  "anomalies": [
    {
      "timestamp": "2025-11-25T14:00:00Z",
      "value": 110,
      "reason": "above_normal"
    }
  ]
}
```

**Implementation**:
```python
# health_analytics.py
from influxdb_client import InfluxDBClient
import pandas as pd
import numpy as np

class HealthAnalyticsService:
    def __init__(self):
        self.influx = InfluxDBClient(
            url="http://localhost:8086",
            token="your_token",
            org="elderly_care"
        )
        self.query_api = self.influx.query_api()

    async def get_health_trends(
        self,
        elderly_id: int,
        metric: str,
        hours: int = 24,
        interval: str = "1h"
    ):
        # Check cache
        cache_key = f"trends:{elderly_id}:{metric}:{hours}:{interval}"
        cached = redis.get(cache_key)
        if cached:
            return json.loads(cached)

        # Query InfluxDB
        query = f'''
        from(bucket: "elderly_care")
          |> range(start: -{hours}h)
          |> filter(fn: (r) => r["_measurement"] == "vital_signs")
          |> filter(fn: (r) => r["elderly_id"] == "{elderly_id}")
          |> filter(fn: (r) => r["_field"] == "{self.get_field_name(metric)}")
          |> aggregateWindow(every: {interval}, fn: mean, createEmpty: false)
          |> yield(name: "mean")
        '''

        result = self.query_api.query(query)

        # Process results
        data_points = []
        for table in result:
            for record in table.records:
                data_points.append({
                    "timestamp": record.get_time().isoformat(),
                    "value": round(record.get_value(), 1),
                    "quality": "good"
                })

        # Calculate statistics
        values = [dp['value'] for dp in data_points]
        statistics = {
            "min": min(values) if values else 0,
            "max": max(values) if values else 0,
            "mean": round(np.mean(values), 1) if values else 0,
            "median": round(np.median(values), 1) if values else 0,
            "stdDev": round(np.std(values), 1) if values else 0
        }

        # Detect anomalies
        anomalies = self.detect_anomalies(data_points, metric)

        result = {
            "metric": metric,
            "unit": self.get_unit(metric),
            "interval": interval,
            "dataPoints": data_points,
            "statistics": statistics,
            "normalRange": self.get_normal_range(metric),
            "anomalies": anomalies
        }

        # Cache for 5 minutes
        redis.setex(cache_key, 300, json.dumps(result))

        return result

    def detect_anomalies(self, data_points, metric):
        """Detect anomalies using statistical methods"""
        anomalies = []
        values = [dp['value'] for dp in data_points]

        if len(values) < 3:
            return anomalies

        mean = np.mean(values)
        std = np.std(values)
        threshold = 3 * std  # 3-sigma rule

        normal_range = self.get_normal_range(metric)

        for dp in data_points:
            value = dp['value']
            reasons = []

            # Check if outside normal physiological range
            if value < normal_range['min']:
                reasons.append('below_normal')
            elif value > normal_range['max']:
                reasons.append('above_normal')

            # Check if statistical outlier
            if abs(value - mean) > threshold:
                reasons.append('statistical_outlier')

            if reasons:
                anomalies.append({
                    "timestamp": dp['timestamp'],
                    "value": value,
                    "reasons": reasons
                })

        return anomalies
```

**Scope**:
- ✅ 24-hour trend chart
- ✅ Switchable metrics (HR, BP, SpO2, Temp)
- ✅ Smooth line chart with Chart.js
- ⚠️ Multiple time ranges (1h, 6h, 24h, 7d, 30d)
- ⚠️ Zoom and pan functionality
- ⚠️ Compare multiple metrics
- ⚠️ Anomaly highlighting
- ⚠️ Export chart as image/PDF
- ⚠️ Statistical analysis overlay

---

## [CONTINUES...]

Would you like me to continue with the remaining features? The document is quite comprehensive. I can create Part 3 covering:

- Medication Schedule (Feature 16)
- AI Insights & Predictions (4 cards) (Feature 17)
- Emergency Contacts (Feature 18)
- Real-time Activity Feed (Feature 19)
- Complete Database Schema
- API Reference Documentation
- Deployment Architecture

Let me know if you'd like me to continue!
