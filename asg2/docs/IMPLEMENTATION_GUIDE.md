# Elderly Care Dashboard - Implementation Guide

## 🎯 Overview
This guide provides step-by-step instructions for implementing the Elderly Care Monitoring Dashboard from prototype to production-ready system.

---

## 📋 Table of Contents
1. [Prerequisites](#prerequisites)
2. [Phase 1: Development Environment Setup](#phase-1-development-environment-setup)
3. [Phase 2: Backend Implementation](#phase-2-backend-implementation)
4. [Phase 3: Frontend Implementation](#phase-3-frontend-implementation)
5. [Phase 4: AI/ML Implementation](#phase-4-aiml-implementation)
6. [Phase 5: IoT Integration](#phase-5-iot-integration)
7. [Phase 6: Robot Implementation](#phase-6-robot-implementation)
8. [Phase 7: Cloud Deployment](#phase-7-cloud-deployment)
9. [Phase 8: Testing & QA](#phase-8-testing--qa)
10. [Phase 9: Production Launch](#phase-9-production-launch)

---

## Prerequisites

### Development Team
```
Minimum Team (MVP):
- 1x Full-stack Developer
- 1x IoT Engineer
- 1x AI/ML Engineer
- 1x UI/UX Designer

Recommended Team (Full Features):
- 2x Full-stack Developers (Frontend + Backend)
- 1x AI/ML Engineer
- 1x IoT/Embedded Engineer
- 1x Robotics Engineer
- 1x UI/UX Designer
- 1x QA Engineer
- 1x Project Manager
```

### Required Skills
- JavaScript/TypeScript (Node.js, React)
- Python (FastAPI, TensorFlow, ROS2)
- SQL (PostgreSQL)
- IoT protocols (MQTT, HTTP, WebSocket)
- Cloud platforms (AWS/Azure/GCP)
- Linux/Ubuntu
- Git version control

### Hardware Requirements (Development)
```
Developer Workstation:
- CPU: Intel i5/i7 or AMD Ryzen 5/7
- RAM: 16GB minimum (32GB recommended)
- Storage: 512GB SSD
- OS: Windows 10/11, macOS, or Linux

IoT Testing Setup:
- 1x Raspberry Pi 4 (8GB)
- 1x ESP32 development board
- 1x Webcam/USB camera
- Various sensors (DHT22, MPU6050, etc.)
- Breadboard, jumper wires

Robot Development (if applicable):
- 1x NVIDIA Jetson Nano/Xavier NX
- 1x RPLIDAR A1
- 1x Robot chassis kit
- 1x Raspberry Pi 4
```

---

## Phase 1: Development Environment Setup

### 1.1 Install Development Tools

#### Windows/Mac/Linux
```bash
# Node.js (v18 LTS)
Download from: https://nodejs.org/

# Python 3.11+
Download from: https://www.python.org/downloads/

# PostgreSQL 15
Download from: https://www.postgresql.org/download/

# MongoDB (optional)
Download from: https://www.mongodb.com/try/download/community

# Redis
Download from: https://redis.io/download/

# Docker Desktop
Download from: https://www.docker.com/products/docker-desktop/

# VS Code (recommended IDE)
Download from: https://code.visualstudio.com/

# Git
Download from: https://git-scm.com/downloads/
```

#### VS Code Extensions
```
- ESLint
- Prettier
- Python
- Docker
- REST Client
- GitLens
- Thunder Client (API testing)
```

### 1.2 Create Project Structure
```bash
elderly-care-system/
├── backend/
│   ├── api-gateway/
│   ├── services/
│   │   ├── auth-service/
│   │   ├── health-service/
│   │   ├── iot-service/
│   │   ├── robot-service/
│   │   └── notification-service/
│   └── shared/
├── frontend/
│   ├── web-dashboard/
│   └── mobile-app/
├── ai-models/
│   ├── fall-detection/
│   ├── health-prediction/
│   └── anomaly-detection/
├── iot-firmware/
│   ├── esp32-sensors/
│   └── raspberry-pi/
├── robot/
│   ├── ros2-packages/
│   └── control-api/
├── infrastructure/
│   ├── docker/
│   ├── kubernetes/
│   └── terraform/
└── docs/
```

### 1.3 Initialize Git Repository
```bash
mkdir elderly-care-system
cd elderly-care-system
git init
git remote add origin <your-repo-url>

# Create .gitignore
cat > .gitignore << EOF
node_modules/
__pycache__/
*.pyc
.env
.DS_Store
dist/
build/
*.log
.vscode/settings.json
EOF
```

---

## Phase 2: Backend Implementation

### 2.1 Setup Node.js Backend

#### Initialize Project
```bash
cd backend/services/auth-service
npm init -y
npm install express typescript @types/node @types/express
npm install jsonwebtoken bcryptjs dotenv cors helmet
npm install --save-dev nodemon ts-node @types/jsonwebtoken
```

#### Create TypeScript Configuration
```json
// tsconfig.json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "commonjs",
    "lib": ["ES2020"],
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules"]
}
```

#### Basic Express Server
```typescript
// src/server.ts
import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import dotenv from 'dotenv';

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;

// Middleware
app.use(helmet());
app.use(cors());
app.use(express.json());

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'healthy', service: 'auth-service' });
});

// Routes
app.use('/api/auth', authRoutes);

app.listen(PORT, () => {
  console.log(`Auth Service running on port ${PORT}`);
});
```

### 2.2 Setup PostgreSQL Database

#### Create Database
```sql
-- Run in PostgreSQL
CREATE DATABASE elderly_care;
CREATE USER elderly_admin WITH ENCRYPTED PASSWORD 'your_password';
GRANT ALL PRIVILEGES ON DATABASE elderly_care TO elderly_admin;
```

#### Setup Sequelize ORM (Node.js)
```bash
npm install sequelize pg pg-hstore
npm install --save-dev sequelize-cli
```

```typescript
// src/config/database.ts
import { Sequelize } from 'sequelize';

export const sequelize = new Sequelize(
  process.env.DB_NAME!,
  process.env.DB_USER!,
  process.env.DB_PASS!,
  {
    host: process.env.DB_HOST,
    dialect: 'postgres',
    logging: false,
  }
);
```

#### Define Models
```typescript
// src/models/User.ts
import { DataTypes, Model } from 'sequelize';
import { sequelize } from '../config/database';

export class User extends Model {
  public id!: number;
  public email!: string;
  public password!: string;
  public role!: 'admin' | 'caregiver' | 'family' | 'doctor';
}

User.init(
  {
    id: {
      type: DataTypes.INTEGER,
      autoIncrement: true,
      primaryKey: true,
    },
    email: {
      type: DataTypes.STRING,
      allowNull: false,
      unique: true,
    },
    password: {
      type: DataTypes.STRING,
      allowNull: false,
    },
    role: {
      type: DataTypes.ENUM('admin', 'caregiver', 'family', 'doctor'),
      allowNull: false,
    },
  },
  {
    sequelize,
    tableName: 'users',
  }
);
```

### 2.3 Setup Python FastAPI Backend (Health Service)

#### Initialize Project
```bash
cd backend/services/health-service
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install fastapi uvicorn sqlalchemy psycopg2-binary pydantic
```

#### Create FastAPI App
```python
# main.py
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import uvicorn

app = FastAPI(title="Health Monitoring Service")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class HealthRecord(BaseModel):
    elderly_id: int
    heart_rate: int
    blood_pressure_sys: int
    blood_pressure_dia: int
    spo2: int
    temperature: float

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "health-monitoring"}

@app.post("/api/health/record")
async def create_health_record(record: HealthRecord):
    # Save to database
    return {"message": "Health record saved", "data": record}

@app.get("/api/health/latest/{elderly_id}")
async def get_latest_health_record(elderly_id: int):
    # Fetch from database
    return {"elderly_id": elderly_id, "heart_rate": 72}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=3002)
```

### 2.4 Setup MQTT Broker (Mosquitto)

#### Install Mosquitto
```bash
# Ubuntu/Debian
sudo apt-get install mosquitto mosquitto-clients

# macOS
brew install mosquitto

# Windows
Download from: https://mosquitto.org/download/
```

#### Configure Mosquitto
```conf
# /etc/mosquitto/mosquitto.conf
listener 1883
allow_anonymous false
password_file /etc/mosquitto/passwd

# WebSocket support
listener 9001
protocol websockets
```

#### Create User
```bash
sudo mosquitto_passwd -c /etc/mosquitto/passwd elderly_system
sudo systemctl restart mosquitto
```

### 2.5 Setup Redis
```bash
# Start Redis
redis-server

# Test connection
redis-cli ping
# Should return: PONG
```

---

## Phase 3: Frontend Implementation

### 3.1 Setup React Dashboard

#### Create React App
```bash
cd frontend/web-dashboard
npx create-react-app . --template typescript
npm install @mui/material @emotion/react @emotion/styled
npm install react-router-dom axios socket.io-client
npm install chart.js react-chartjs-2
npm install @reduxjs/toolkit react-redux
```

#### Project Structure
```
src/
├── components/
│   ├── common/
│   ├── dashboard/
│   ├── health/
│   ├── robot/
│   └── iot/
├── pages/
│   ├── Dashboard.tsx
│   ├── HealthAnalytics.tsx
│   ├── RobotControl.tsx
│   └── Settings.tsx
├── services/
│   ├── api.ts
│   ├── socket.ts
│   └── auth.ts
├── store/
│   ├── slices/
│   └── store.ts
├── types/
│   └── index.ts
└── App.tsx
```

#### API Service
```typescript
// src/services/api.ts
import axios from 'axios';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:3000';

export const api = axios.create({
  baseURL: API_BASE_URL,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Add auth token to requests
api.interceptors.request.use((config) => {
  const token = localStorage.getItem('token');
  if (token) {
    config.headers.Authorization = `Bearer ${token}`;
  }
  return config;
});

// Health API
export const healthAPI = {
  getLatest: (elderlyId: number) =>
    api.get(`/api/health/latest/${elderlyId}`),

  getHistory: (elderlyId: number, days: number) =>
    api.get(`/api/health/history/${elderlyId}?days=${days}`),
};
```

#### WebSocket Connection
```typescript
// src/services/socket.ts
import { io, Socket } from 'socket.io-client';

class SocketService {
  private socket: Socket | null = null;

  connect(userId: number) {
    this.socket = io(process.env.REACT_APP_SOCKET_URL || 'http://localhost:3000', {
      auth: { userId },
    });

    this.socket.on('connect', () => {
      console.log('Socket connected');
    });

    this.socket.on('health_update', (data) => {
      console.log('Health update:', data);
      // Dispatch to Redux store
    });

    this.socket.on('alert', (data) => {
      console.log('Alert:', data);
      // Show notification
    });
  }

  disconnect() {
    if (this.socket) {
      this.socket.disconnect();
    }
  }
}

export default new SocketService();
```

#### Redux Store Setup
```typescript
// src/store/store.ts
import { configureStore } from '@reduxjs/toolkit';
import healthReducer from './slices/healthSlice';
import authReducer from './slices/authSlice';

export const store = configureStore({
  reducer: {
    health: healthReducer,
    auth: authReducer,
  },
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;
```

### 3.2 Implement Dashboard Components

#### Health Card Component
```typescript
// src/components/dashboard/HealthCard.tsx
import React from 'react';
import { Card, CardContent, Typography } from '@mui/material';
import { Favorite } from '@mui/icons-material';

interface HealthCardProps {
  title: string;
  value: string;
  icon: React.ReactNode;
  status: 'normal' | 'warning' | 'critical';
}

const HealthCard: React.FC<HealthCardProps> = ({ title, value, icon, status }) => {
  const getColor = () => {
    switch (status) {
      case 'normal': return '#27AE60';
      case 'warning': return '#F39C12';
      case 'critical': return '#E74C3C';
    }
  };

  return (
    <Card sx={{ minWidth: 200 }}>
      <CardContent>
        <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
          <div style={{ color: getColor(), fontSize: 40 }}>
            {icon}
          </div>
          <div>
            <Typography color="textSecondary" gutterBottom>
              {title}
            </Typography>
            <Typography variant="h4" style={{ color: getColor() }}>
              {value}
            </Typography>
          </div>
        </div>
      </CardContent>
    </Card>
  );
};

export default HealthCard;
```

---

## Phase 4: AI/ML Implementation

### 4.1 Fall Detection Model

#### Setup Environment
```bash
cd ai-models/fall-detection
python -m venv venv
source venv/bin/activate
pip install tensorflow opencv-python numpy ultralytics
```

#### Train YOLOv8 Pose Model
```python
# train_fall_detection.py
from ultralytics import YOLO
import cv2
import numpy as np

# Load pre-trained YOLOv8 pose model
model = YOLO('yolov8n-pose.pt')

# Fine-tune on fall detection dataset
# (Assume you have a dataset with fall/non-fall videos)
model.train(
    data='fall_dataset.yaml',
    epochs=100,
    imgsz=640,
    device='cuda'  # or 'cpu'
)

# Save model
model.save('fall_detection_model.pt')
```

#### Inference Script
```python
# detect_fall.py
import cv2
import numpy as np
from ultralytics import YOLO
import time

class FallDetector:
    def __init__(self, model_path='fall_detection_model.pt'):
        self.model = YOLO(model_path)
        self.fall_threshold = 0.85

    def detect_fall(self, frame):
        """
        Detect fall from video frame
        Returns: (is_fall, confidence, keypoints)
        """
        results = self.model(frame)

        if len(results) == 0:
            return False, 0.0, None

        # Get pose keypoints
        keypoints = results[0].keypoints.xy[0]  # First person

        # Calculate features for fall detection
        is_fall, confidence = self._classify_fall(keypoints)

        return is_fall, confidence, keypoints

    def _classify_fall(self, keypoints):
        """
        Classify if pose indicates a fall
        Based on body angle, velocity, etc.
        """
        # Example: Check if body is horizontal
        # (This is simplified - actual implementation would be more complex)

        if len(keypoints) < 17:  # COCO pose has 17 keypoints
            return False, 0.0

        # Calculate body angle
        shoulder = keypoints[5]  # Right shoulder
        hip = keypoints[11]  # Right hip

        angle = np.arctan2(hip[1] - shoulder[1], hip[0] - shoulder[0])
        angle_deg = np.degrees(angle)

        # If body is nearly horizontal, might be a fall
        if abs(angle_deg) < 30:
            confidence = 0.9
            return True, confidence

        return False, 0.0

# Usage
detector = FallDetector()
cap = cv2.VideoCapture(0)  # Webcam

while True:
    ret, frame = cap.read()
    if not ret:
        break

    is_fall, confidence, keypoints = detector.detect_fall(frame)

    if is_fall:
        print(f"FALL DETECTED! Confidence: {confidence:.2f}")
        # Send alert via API

    cv2.imshow('Fall Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### 4.2 Health Prediction Model

#### Data Preparation
```python
# prepare_data.py
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler

# Load historical health data
data = pd.read_csv('health_records.csv')

# Features: heart_rate, bp_sys, bp_dia, spo2, temp, steps, sleep_hours, medication_adherence
features = ['heart_rate', 'bp_sys', 'bp_dia', 'spo2', 'temperature',
            'steps', 'sleep_hours', 'medication_adherence']

# Create sequences (30 days of data)
def create_sequences(data, seq_length=30):
    X, y = [], []
    for i in range(len(data) - seq_length):
        X.append(data[i:i+seq_length, :])
        y.append(data[i+seq_length, -1])  # Next day's health status
    return np.array(X), np.array(y)

X, y = create_sequences(data[features].values)

# Split data
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
```

#### Build LSTM Model
```python
# train_health_prediction.py
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

# Build LSTM model
model = keras.Sequential([
    layers.LSTM(128, return_sequences=True, input_shape=(30, 8)),  # 30 days, 8 features
    layers.Dropout(0.3),
    layers.LSTM(64),
    layers.Dropout(0.3),
    layers.Dense(32, activation='relu'),
    layers.Dense(1, activation='sigmoid')  # Health stability score (0-1)
])

model.compile(
    optimizer='adam',
    loss='binary_crossentropy',
    metrics=['accuracy', 'AUC']
)

# Train
history = model.fit(
    X_train, y_train,
    validation_data=(X_test, y_test),
    epochs=50,
    batch_size=32
)

# Save model
model.save('health_prediction_model.h5')
```

#### Prediction API
```python
# prediction_service.py
from fastapi import FastAPI
import tensorflow as tf
import numpy as np

app = FastAPI()

# Load model
model = tf.keras.models.load_model('health_prediction_model.h5')

@app.post("/predict")
async def predict_health_stability(data: dict):
    """
    Input: 30 days of health data
    Output: Health stability prediction (0-100%)
    """
    # Preprocess input
    features = np.array(data['features']).reshape(1, 30, 8)

    # Predict
    prediction = model.predict(features)[0][0]

    return {
        "stability_score": float(prediction * 100),
        "status": "stable" if prediction > 0.7 else "at_risk"
    }
```

---

## Phase 5: IoT Integration

### 5.1 ESP32 Firmware (Wearable Sensor)

#### Setup Arduino IDE
```
1. Install Arduino IDE: https://www.arduino.cc/en/software
2. Add ESP32 board: File → Preferences → Additional Board Manager URLs:
   https://dl.espressif.com/dl/package_esp32_index.json
3. Install ESP32 board: Tools → Board Manager → ESP32
4. Install libraries:
   - PubSubClient (MQTT)
   - ArduinoJson
   - MAX30102 (heart rate sensor)
   - MPU6050 (accelerometer)
```

#### ESP32 Code (Heart Rate Monitor)
```cpp
// esp32_heart_rate.ino
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker
const char* mqtt_server = "your-mqtt-broker.com";
const int mqtt_port = 1883;
const char* mqtt_user = "elderly_system";
const char* mqtt_pass = "your_password";

// Device ID
const char* device_id = "elderly_001_smartwatch";

WiFiClient espClient;
PubSubClient client(espClient);
MAX30105 particleSensor;

// Heart rate variables
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Connect to MQTT
  client.setServer(mqtt_server, mqtt_port);
  reconnect();

  // Initialize MAX30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read heart rate
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      // Calculate average
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // Publish data every 5 seconds
  static unsigned long lastPublish = 0;
  if (millis() - lastPublish > 5000) {
    publishHealthData();
    lastPublish = millis();
  }
}

void publishHealthData() {
  // Create JSON payload
  String payload = "{";
  payload += "\"device_id\":\"" + String(device_id) + "\",";
  payload += "\"timestamp\":" + String(millis()) + ",";
  payload += "\"heart_rate\":" + String(beatAvg) + ",";
  payload += "\"spo2\":98";  // From SpO2 sensor
  payload += "}";

  // Publish to MQTT topic
  client.publish("elderly/health/vital_signs", payload.c_str());

  Serial.println("Published: " + payload);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(device_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}
```

### 5.2 Raspberry Pi - Home Gateway

#### Install Dependencies
```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install Node.js
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install Python
sudo apt-get install -y python3 python3-pip

# Install Mosquitto MQTT Broker
sudo apt-get install -y mosquitto mosquitto-clients

# Install Docker (optional)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

#### MQTT to HTTP Bridge (Node.js)
```javascript
// mqtt_bridge.js
const mqtt = require('mqtt');
const axios = require('axios');

const MQTT_BROKER = 'mqtt://localhost:1883';
const API_URL = 'https://your-api.com';

// Connect to MQTT broker
const client = mqtt.connect(MQTT_BROKER, {
  username: 'elderly_system',
  password: 'your_password'
});

client.on('connect', () => {
  console.log('Connected to MQTT broker');

  // Subscribe to topics
  client.subscribe('elderly/health/#');
  client.subscribe('elderly/iot/#');
  client.subscribe('elderly/alerts/#');
});

client.on('message', async (topic, message) => {
  console.log(`Received message on ${topic}`);

  try {
    const data = JSON.parse(message.toString());

    // Forward to HTTP API
    await axios.post(`${API_URL}/api/iot/telemetry`, {
      topic: topic,
      data: data,
      timestamp: new Date().toISOString()
    });

    console.log('Data forwarded to API');
  } catch (error) {
    console.error('Error processing message:', error);
  }
});
```

---

## Phase 6: Robot Implementation

### 6.1 Setup ROS2 on Raspberry Pi

#### Install ROS2 Humble
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Create ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create robot package
ros2 pkg create --build-type ament_python elderly_care_robot
cd elderly_care_robot
```

#### Basic Robot Node
```python
# elderly_care_robot/elderly_care_robot/robot_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import json
import paho.mqtt.client as mqtt

class ElderCareRobot(Node):
    def __init__(self):
        super().__init__('elder_care_robot')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("localhost", 1883)
        self.mqtt_client.on_message = self.mqtt_callback
        self.mqtt_client.subscribe("robot/commands")
        self.mqtt_client.loop_start()

        self.get_logger().info('Elder Care Robot Node Started')

    def mqtt_callback(self, client, userdata, msg):
        """Handle commands from MQTT"""
        try:
            command = json.loads(msg.payload.decode())
            action = command.get('action')

            if action == 'move_forward':
                self.move_forward()
            elif action == 'turn_left':
                self.turn_left()
            elif action == 'stop':
                self.stop()

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.2  # meters/second
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Moving forward')

    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # radians/second
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Turning left')

    def stop(self):
        """Stop robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Stopped')

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection"""
        # Check for obstacles in front
        front_distance = min(msg.ranges[0:30])  # Front 30 degrees

        if front_distance < 0.5:  # Less than 50cm
            self.stop()
            self.get_logger().warn(f'Obstacle detected at {front_distance}m')

def main(args=None):
    rclpy.init(args=args)
    robot = ElderCareRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Phase 7: Cloud Deployment

### 7.1 Setup AWS Account

1. Create AWS Account: https://aws.amazon.com/
2. Create IAM user with appropriate permissions
3. Install AWS CLI: `pip install awscli`
4. Configure CLI: `aws configure`

### 7.2 Deploy with Docker + ECS

#### Create Dockerfile (Backend)
```dockerfile
# Dockerfile
FROM node:18-alpine

WORKDIR /app

COPY package*.json ./
RUN npm ci --only=production

COPY . .

EXPOSE 3000

CMD ["npm", "start"]
```

#### Build and Push to ECR
```bash
# Create ECR repository
aws ecr create-repository --repository-name elderly-care-api

# Login to ECR
aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin <your-account-id>.dkr.ecr.us-east-1.amazonaws.com

# Build image
docker build -t elderly-care-api .

# Tag image
docker tag elderly-care-api:latest <your-account-id>.dkr.ecr.us-east-1.amazonaws.com/elderly-care-api:latest

# Push image
docker push <your-account-id>.dkr.ecr.us-east-1.amazonaws.com/elderly-care-api:latest
```

### 7.3 Setup RDS (PostgreSQL)
```bash
# Via AWS CLI
aws rds create-db-instance \
    --db-instance-identifier elderly-care-db \
    --db-instance-class db.t3.medium \
    --engine postgres \
    --engine-version 15.3 \
    --master-username admin \
    --master-user-password YourPassword123 \
    --allocated-storage 100 \
    --vpc-security-group-ids sg-xxxxx
```

### 7.4 Setup AWS IoT Core
```bash
# Create IoT Thing
aws iot create-thing --thing-name elderly-001-smartwatch

# Create certificate
aws iot create-keys-and-certificate \
    --set-as-active \
    --certificate-pem-outfile certificate.pem \
    --public-key-outfile public.key \
    --private-key-outfile private.key

# Attach policy to certificate
aws iot attach-policy \
    --policy-name ElderlyDevicePolicy \
    --target <certificate-arn>
```

---

## Phase 8: Testing & QA

### 8.1 Unit Testing (Backend)

#### Jest (Node.js)
```typescript
// __tests__/auth.test.ts
import request from 'supertest';
import app from '../src/app';

describe('Auth API', () => {
  it('should register a new user', async () => {
    const res = await request(app)
      .post('/api/auth/register')
      .send({
        email: 'test@example.com',
        password: 'Password123!',
        role: 'caregiver'
      });

    expect(res.status).toBe(201);
    expect(res.body).toHaveProperty('token');
  });

  it('should login existing user', async () => {
    const res = await request(app)
      .post('/api/auth/login')
      .send({
        email: 'test@example.com',
        password: 'Password123!'
      });

    expect(res.status).toBe(200);
    expect(res.body).toHaveProperty('token');
  });
});
```

### 8.2 Integration Testing

#### Pytest (Python)
```python
# tests/test_health_api.py
import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_create_health_record():
    response = client.post("/api/health/record", json={
        "elderly_id": 1,
        "heart_rate": 75,
        "blood_pressure_sys": 120,
        "blood_pressure_dia": 80,
        "spo2": 98,
        "temperature": 36.8
    })
    assert response.status_code == 200
    assert response.json()["message"] == "Health record saved"

def test_get_latest_health_record():
    response = client.get("/api/health/latest/1")
    assert response.status_code == 200
    assert "heart_rate" in response.json()
```

### 8.3 E2E Testing (Frontend)

#### Cypress
```javascript
// cypress/e2e/dashboard.cy.ts
describe('Dashboard', () => {
  beforeEach(() => {
    // Login
    cy.visit('/login');
    cy.get('[data-cy=email]').type('test@example.com');
    cy.get('[data-cy=password]').type('Password123!');
    cy.get('[data-cy=login-btn]').click();

    cy.url().should('include', '/dashboard');
  });

  it('should display health metrics', () => {
    cy.get('[data-cy=heart-rate]').should('exist');
    cy.get('[data-cy=blood-pressure]').should('exist');
    cy.get('[data-cy=oxygen]').should('exist');
  });

  it('should navigate to robot control', () => {
    cy.get('[data-cy=nav-robot]').click();
    cy.url().should('include', '/robot-control');
  });
});
```

---

## Phase 9: Production Launch

### 9.1 Pre-Launch Checklist

```
✅ Security
  - [ ] All API endpoints have authentication
  - [ ] HTTPS enabled (SSL certificates)
  - [ ] Environment variables secured
  - [ ] Database backups automated
  - [ ] Security audit completed
  - [ ] Penetration testing done

✅ Performance
  - [ ] Load testing completed (1000+ concurrent users)
  - [ ] Database indexed properly
  - [ ] CDN configured for static assets
  - [ ] Caching implemented (Redis)
  - [ ] Image optimization done

✅ Monitoring
  - [ ] Logging configured (ELK/CloudWatch)
  - [ ] Error tracking (Sentry)
  - [ ] Uptime monitoring (Pingdom)
  - [ ] Performance monitoring (New Relic)
  - [ ] Alerts configured (PagerDuty)

✅ Documentation
  - [ ] API documentation (Swagger)
  - [ ] User manual created
  - [ ] Admin guide created
  - [ ] Deployment guide created

✅ Legal & Compliance
  - [ ] Privacy policy published
  - [ ] Terms of service published
  - [ ] PDPA compliance verified
  - [ ] Medical device registration (if required)

✅ Support
  - [ ] Help desk system ready
  - [ ] FAQ created
  - [ ] Support team trained
  - [ ] Escalation procedures defined
```

### 9.2 Deployment Steps

```bash
# 1. Final build
npm run build  # Frontend
docker build -t elderly-care-api:v1.0.0 .  # Backend

# 2. Database migration
npm run migrate  # or alembic upgrade head

# 3. Deploy to production
kubectl apply -f k8s/production/  # Kubernetes
# or
aws ecs update-service --cluster prod --service elderly-care-api  # ECS

# 4. Verify deployment
curl https://api.eldercare.com/health
```

### 9.3 Post-Launch Monitoring

```bash
# Monitor logs
kubectl logs -f deployment/elderly-care-api
# or
aws logs tail /aws/ecs/elderly-care-api --follow

# Check metrics
kubectl top pods
# or
aws cloudwatch get-metric-statistics --metric-name CPUUtilization
```

---

## 🎉 Congratulations!

You've successfully implemented the Elderly Care Monitoring Dashboard!

### Next Steps:
1. Gather user feedback
2. Iterate on features
3. Scale infrastructure as needed
4. Add more AI models
5. Expand to more cities

---

**Document Version**: 1.0
**Last Updated**: November 25, 2025
**For**: Development Team
