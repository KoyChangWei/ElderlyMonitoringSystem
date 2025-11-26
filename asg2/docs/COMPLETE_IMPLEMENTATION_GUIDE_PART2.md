# Complete Implementation Guide Part 2 - Elderly Care Dashboard
## Remaining Features: Detailed Backend Logic, AI Models & Tech Stack

**Document Version:** 2.0 - Part 2
**Last Updated:** November 25, 2025
**Course:** STTHK3133 Human Factor Engineering

---

## Table of Contents (Part 2)

17. [Hydration Tracker](#17-hydration-tracker)
18. [GPS Location & Geofencing](#18-gps-location--geofencing)
19. [Wearable Devices Integration](#19-wearable-devices-integration)
20. [Environmental Monitoring](#20-environmental-monitoring)
21. [Voice Control Assistant](#21-voice-control-assistant)
22. [Smart Automation Rules Engine](#22-smart-automation-rules-engine)
23. [Complete System Integration](#23-complete-system-integration)
24. [Deployment Architecture](#24-deployment-architecture)
25. [Security & Compliance](#25-security--compliance)

---

# 17. Hydration Tracker

## Purpose & Use Case
Monitor daily water intake for elderly to prevent dehydration, which is common and dangerous for seniors. Track consumption patterns and send reminders.

## Frontend Implementation
```javascript
// Hydration tracking interface
let waterGlasses = 0;
const dailyGoal = 8; // 8 glasses (2000ml)

function addWaterGlass() {
    waterGlasses++;
    updateHydrationDisplay();
    sendToBackend();
}

function updateHydrationDisplay() {
    const glassElements = document.querySelectorAll('.water-glass');
    glassElements.forEach((glass, index) => {
        glass.classList.toggle('filled', index < waterGlasses);
    });

    document.getElementById('waterCount').textContent =
        `${waterGlasses} / ${dailyGoal} glasses`;
    document.getElementById('waterML').textContent =
        `${waterGlasses * 250}ml / 2,000ml`;

    // Update progress bar
    const percentage = (waterGlasses / dailyGoal) * 100;
    document.getElementById('hydrationProgress').style.width = `${percentage}%`;
}

async function sendToBackend() {
    await fetch('/api/hydration/log', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            elderly_id: currentElderlyId,
            amount_ml: 250,
            timestamp: new Date().toISOString()
        })
    });
}
```

## Backend Logic
**Service:** Hydration Monitoring Service (Node.js + PostgreSQL)

**Implementation:**
```javascript
// Node.js + Express
const express = require('express');
const { Pool } = require('pg');
const cron = require('node-cron');

const router = express.Router();
const pool = new Pool({ connectionString: process.env.DATABASE_URL });

// POST - Log water intake
router.post('/api/hydration/log', async (req, res) => {
    const { elderly_id, amount_ml, timestamp, source } = req.body;

    try {
        // Insert hydration record
        const result = await pool.query(`
            INSERT INTO hydration_logs (elderly_id, amount_ml, recorded_at, source)
            VALUES ($1, $2, $3, $4)
            RETURNING *
        `, [elderly_id, amount_ml, timestamp || new Date(), source || 'manual']);

        // Check if daily goal reached
        const today = await getDailyHydration(elderly_id);

        if (today.total_ml >= today.goal_ml && !today.goal_reached_notified) {
            await sendGoalAchievedNotification(elderly_id);
            await markGoalReached(elderly_id);
        }

        // Update real-time stats
        await updateRedisStats(elderly_id, amount_ml);

        res.json({
            success: true,
            record: result.rows[0],
            daily_total: today.total_ml,
            daily_goal: today.goal_ml,
            percentage: Math.round((today.total_ml / today.goal_ml) * 100)
        });

    } catch (error) {
        console.error('Hydration log error:', error);
        res.status(500).json({ error: 'Failed to log hydration' });
    }
});

// GET - Daily hydration status
router.get('/api/hydration/:elderlyId/today', async (req, res) => {
    const { elderlyId } = req.params;

    try {
        const stats = await getDailyHydration(elderlyId);
        res.json(stats);
    } catch (error) {
        res.status(500).json({ error: 'Failed to fetch hydration data' });
    }
});

// GET - Hydration history
router.get('/api/hydration/:elderlyId/history', async (req, res) => {
    const { elderlyId } = req.params;
    const { days = 7 } = req.query;

    try {
        const history = await pool.query(`
            SELECT
                DATE(recorded_at) as date,
                SUM(amount_ml) as total_ml,
                COUNT(*) as intake_count,
                hg.goal_ml,
                ROUND((SUM(amount_ml)::decimal / hg.goal_ml) * 100, 1) as percentage
            FROM hydration_logs hl
            LEFT JOIN hydration_goals hg ON hl.elderly_id = hg.elderly_id
            WHERE hl.elderly_id = $1
                AND recorded_at >= CURRENT_DATE - INTERVAL '${days} days'
            GROUP BY DATE(recorded_at), hg.goal_ml
            ORDER BY date DESC
        `, [elderlyId]);

        res.json({
            elderly_id: elderlyId,
            period_days: days,
            history: history.rows,
            average_daily: calculateAverage(history.rows),
            adherence_rate: calculateAdherence(history.rows)
        });

    } catch (error) {
        res.status(500).json({ error: 'Failed to fetch history' });
    }
});

async function getDailyHydration(elderlyId) {
    const result = await pool.query(`
        SELECT
            COALESCE(SUM(hl.amount_ml), 0) as total_ml,
            COUNT(*) as intake_count,
            hg.goal_ml,
            hg.goal_reached_today
        FROM hydration_goals hg
        LEFT JOIN hydration_logs hl ON hg.elderly_id = hl.elderly_id
            AND DATE(hl.recorded_at) = CURRENT_DATE
        WHERE hg.elderly_id = $1
        GROUP BY hg.goal_ml, hg.goal_reached_today
    `, [elderlyId]);

    if (result.rows.length === 0) {
        // Create default goal if not exists
        await pool.query(`
            INSERT INTO hydration_goals (elderly_id, goal_ml)
            VALUES ($1, 2000)
            ON CONFLICT (elderly_id) DO NOTHING
        `, [elderlyId]);

        return { total_ml: 0, goal_ml: 2000, intake_count: 0, goal_reached_notified: false };
    }

    return result.rows[0];
}

// Automated reminder system
cron.schedule('0 9,12,15,18,21 * * *', async () => {
    // Send reminders at 9am, 12pm, 3pm, 6pm, 9pm
    console.log('Running hydration reminder check...');

    const elderly = await pool.query(`
        SELECT
            ep.id,
            ep.name,
            COALESCE(SUM(hl.amount_ml), 0) as today_ml,
            hg.goal_ml,
            EXTRACT(HOUR FROM NOW()) as current_hour
        FROM elderly_profiles ep
        JOIN hydration_goals hg ON ep.id = hg.elderly_id
        LEFT JOIN hydration_logs hl ON ep.id = hl.elderly_id
            AND DATE(hl.recorded_at) = CURRENT_DATE
        GROUP BY ep.id, ep.name, hg.goal_ml
        HAVING COALESCE(SUM(hl.amount_ml), 0) < hg.goal_ml
    `);

    for (const person of elderly.rows) {
        const expectedByNow = calculateExpectedIntake(person.current_hour, person.goal_ml);

        if (person.today_ml < expectedByNow) {
            await sendHydrationReminder(person.id, person.name, person.today_ml, person.goal_ml);
        }
    }
});

function calculateExpectedIntake(hour, dailyGoal) {
    // Expected intake increases throughout the day
    // 9am: 25%, 12pm: 37.5%, 3pm: 50%, 6pm: 75%, 9pm: 87.5%
    const schedule = {
        9: 0.25,
        12: 0.375,
        15: 0.50,
        18: 0.75,
        21: 0.875
    };

    const percentage = schedule[hour] || 0.5;
    return dailyGoal * percentage;
}

async function sendHydrationReminder(elderlyId, name, currentMl, goalMl) {
    const remaining = goalMl - currentMl;

    // Send push notification
    const message = {
        title: '💧 Hydration Reminder',
        body: `Time to drink water! You need ${remaining}ml more today.`,
        data: {
            type: 'hydration_reminder',
            elderly_id: elderlyId
        }
    };

    // Publish to notification service
    const redis = require('./redis').client;
    await redis.publish('notifications:push', JSON.stringify(message));

    // If severely behind, also notify caregiver
    if (currentMl < goalMl * 0.3 && new Date().getHours() >= 15) {
        await notifyCaregiver(elderlyId, `${name} has only consumed ${currentMl}ml of water today`);
    }
}

module.exports = router;
```

## Hardware Integration - Smart Water Bottle

### IoT Smart Water Bottle (HidrateSpark or similar)
```python
# Smart water bottle with sensors
# Measures: volume consumed, sip frequency, bottle tilt
# Connection: Bluetooth Low Energy

import asyncio
from bleak import BleakClient
import requests

BOTTLE_MAC = "AA:BB:CC:DD:EE:FF"
BOTTLE_UUID_VOLUME = "12345678-1234-5678-1234-56789abcdef0"

async def connect_smart_bottle():
    """Connect to smart water bottle via BLE"""
    async with BleakClient(BOTTLE_MAC) as client:
        print(f"Connected to bottle: {client.is_connected}")

        # Subscribe to volume notifications
        await client.start_notify(BOTTLE_UUID_VOLUME, volume_callback)

        # Keep connection alive
        while True:
            await asyncio.sleep(1)

def volume_callback(sender, data):
    """Called when water volume changes (sip detected)"""
    # Parse BLE data
    volume_consumed_ml = int.from_bytes(data[0:2], byteorder='little')

    print(f"Sip detected: {volume_consumed_ml}ml")

    # Send to cloud
    response = requests.post(
        "https://api.eldercare.com/api/hydration/log",
        json={
            "elderly_id": ELDERLY_ID,
            "amount_ml": volume_consumed_ml,
            "source": "smart_bottle",
            "timestamp": datetime.utcnow().isoformat()
        },
        headers={"Authorization": f"Bearer {API_TOKEN}"}
    )

    print(f"Cloud sync: {response.status_code}")

# Run
asyncio.run(connect_smart_bottle())
```

## Database Schema
```sql
CREATE TABLE hydration_logs (
    id SERIAL PRIMARY KEY,
    elderly_id INTEGER REFERENCES elderly_profiles(id),
    amount_ml INTEGER NOT NULL,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    source VARCHAR(50) DEFAULT 'manual', -- manual, smart_bottle, voice_assistant
    temperature VARCHAR(20), -- hot, warm, cold (optional)
    beverage_type VARCHAR(50) DEFAULT 'water' -- water, tea, juice, etc.
);

CREATE TABLE hydration_goals (
    id SERIAL PRIMARY KEY,
    elderly_id INTEGER REFERENCES elderly_profiles(id) UNIQUE,
    goal_ml INTEGER DEFAULT 2000,
    goal_reached_today BOOLEAN DEFAULT false,
    last_reminder_sent TIMESTAMP,
    reminder_frequency_hours INTEGER DEFAULT 3,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE hydration_reminders (
    id SERIAL PRIMARY KEY,
    elderly_id INTEGER REFERENCES elderly_profiles(id),
    reminder_time TIME NOT NULL, -- e.g., 09:00, 12:00, 15:00, 18:00, 21:00
    enabled BOOLEAN DEFAULT true
);

CREATE INDEX idx_hydration_logs_elderly_date ON hydration_logs(elderly_id, DATE(recorded_at));
CREATE INDEX idx_hydration_logs_today ON hydration_logs(elderly_id, recorded_at)
    WHERE DATE(recorded_at) = CURRENT_DATE;
```

## AI/ML Component - Personalized Hydration Goals

```python
# Adjust hydration goals based on activity, weather, health conditions
from sklearn.ensemble import RandomForestRegressor
import pandas as pd

class PersonalizedHydrationCalculator:
    def __init__(self):
        self.model = RandomForestRegressor(n_estimators=100)

    def calculate_daily_goal(self, elderly_profile, weather, activity_level):
        """
        Calculate personalized hydration goal
        """
        # Base calculation (standard formula)
        weight_kg = elderly_profile['weight_kg']
        base_ml = weight_kg * 30  # 30ml per kg body weight

        # Adjustments
        factors = {
            'age': self._age_factor(elderly_profile['age']),
            'activity': self._activity_factor(activity_level),
            'weather': self._weather_factor(weather['temperature'], weather['humidity']),
            'health': self._health_factor(elderly_profile['conditions']),
            'medications': self._medication_factor(elderly_profile['medications'])
        }

        # Apply factors
        adjusted_ml = base_ml
        for factor_name, multiplier in factors.items():
            adjusted_ml *= multiplier

        # Ensure within safe range
        adjusted_ml = max(1500, min(3000, adjusted_ml))

        return int(adjusted_ml)

    def _age_factor(self, age):
        """Elderly have reduced thirst sensation"""
        if age >= 80:
            return 1.15  # Need more due to reduced kidney function
        elif age >= 70:
            return 1.10
        return 1.0

    def _activity_factor(self, steps):
        """More active = more hydration needed"""
        if steps > 8000:
            return 1.3
        elif steps > 5000:
            return 1.2
        elif steps > 3000:
            return 1.1
        return 1.0

    def _weather_factor(self, temp_c, humidity):
        """Hot weather = more hydration"""
        if temp_c > 32:
            return 1.4
        elif temp_c > 28:
            return 1.2
        elif temp_c > 24:
            return 1.1
        return 1.0

    def _health_factor(self, conditions):
        """Certain conditions require more/less water"""
        multiplier = 1.0

        if 'diabetes' in conditions:
            multiplier *= 1.2  # Diabetes increases thirst
        if 'kidney_disease' in conditions:
            multiplier *= 0.8  # May need restriction
        if 'heart_failure' in conditions:
            multiplier *= 0.85  # Fluid restriction often needed

        return multiplier

    def _medication_factor(self, medications):
        """Some medications increase hydration needs"""
        multiplier = 1.0

        diuretic_meds = ['furosemide', 'hydrochlorothiazide']
        if any(med in medications for med in diuretic_meds):
            multiplier *= 1.25  # Diuretics increase fluid loss

        return multiplier
```

## Tech Stack
- **Backend:** Node.js + Express.js
- **Database:** PostgreSQL 15
- **Scheduling:** node-cron
- **Hardware:** HidrateSpark Smart Water Bottle / Thermos Smart Cup
- **Communication:** Bluetooth LE
- **Notifications:** AWS SNS, FCM (Firebase Cloud Messaging)

## Scope
- **Basic:** Manual logging, daily goal tracking
- **Advanced:** Smart bottle integration, automated reminders, personalized goals
- **Future:** Urine analysis integration, dehydration prediction, IV fluid tracking for hospitalized patients

---

# 18. GPS Location & Geofencing

## Purpose & Use Case
Track elderly location in real-time for safety, especially those with dementia/wandering tendencies. Alert caregivers when elderly leaves safe zones (geofences).

## Frontend Implementation
```javascript
// Google Maps integration with geofencing
let map;
let elderlyMarker;
let geofenceCircle;

async function initializeMap() {
    const { elderlyId } = getCurrentElderly();

    // Load Google Maps
    map = new google.maps.Map(document.getElementById('map'), {
        center: { lat: 3.1390, lng: 101.6869 }, // Kuala Lumpur
        zoom: 15
    });

    // Load current location
    const location = await fetchCurrentLocation(elderlyId);
    updateElderlyPosition(location);

    // Load geofence
    const geofence = await fetchGeofence(elderlyId);
    drawGeofence(geofence);

    // Start real-time tracking
    startLocationTracking(elderlyId);
}

function updateElderlyPosition(location) {
    if (!elderlyMarker) {
        elderlyMarker = new google.maps.Marker({
            position: { lat: location.latitude, lng: location.longitude },
            map: map,
            title: 'Current Location',
            icon: {
                url: '/images/elderly-marker.png',
                scaledSize: new google.maps.Size(40, 40)
            }
        });
    } else {
        elderlyMarker.setPosition({ lat: location.latitude, lng: location.longitude });
    }

    // Check if outside geofence
    if (geofenceCircle) {
        const distance = google.maps.geometry.spherical.computeDistanceBetween(
            elderlyMarker.getPosition(),
            geofenceCircle.getCenter()
        );

        if (distance > geofenceCircle.getRadius()) {
            showGeofenceAlert(location);
        }
    }
}

function drawGeofence(geofence) {
    geofenceCircle = new google.maps.Circle({
        strokeColor: '#4A90E2',
        strokeOpacity: 0.8,
        strokeWeight: 2,
        fillColor: '#4A90E2',
        fillOpacity: 0.15,
        map: map,
        center: { lat: geofence.center_lat, lng: geofence.center_lng },
        radius: geofence.radius_meters
    });
}

function startLocationTracking(elderlyId) {
    const ws = new WebSocket(`wss://api.eldercare.com/location/stream/${elderlyId}`);

    ws.onmessage = (event) => {
        const location = JSON.parse(event.data);
        updateElderlyPosition(location);
        updateLocationHistory(location);
    };
}
```

## Backend Logic
**Service:** GPS Tracking Service (Python + PostGIS)

**Implementation:**
```python
# Python FastAPI + PostGIS
from fastapi import FastAPI, WebSocket, BackgroundTasks
from sqlalchemy import create_engine, Column, Integer, Float, DateTime, Boolean
from geoalchemy2 import Geography
from datetime import datetime, timedelta
import asyncio

app = FastAPI()

# Database with PostGIS extension
engine = create_engine(os.getenv("DATABASE_URL"))

@app.post("/api/location/update")
async def update_location(location_data: LocationUpdate):
    """
    Receive GPS location from wearable device
    """
    elderly_id = location_data.elderly_id
    lat = location_data.latitude
    lng = location_data.longitude
    accuracy = location_data.accuracy
    timestamp = location_data.timestamp or datetime.utcnow()

    # Store location in PostGIS
    await db.execute(f'''
        INSERT INTO location_history (
            elderly_id, location, latitude, longitude, accuracy,
            recorded_at, battery_level, source
        )
        VALUES (
            {elderly_id},
            ST_SetSRID(ST_MakePoint({lng}, {lat}), 4326),
            {lat}, {lng}, {accuracy}, '{timestamp}',
            {location_data.battery_level}, '{location_data.source}'
        )
    ''')

    # Check geofence violations
    geofence_status = await check_geofence_violation(elderly_id, lat, lng)

    if geofence_status['violated']:
        await trigger_geofence_alert(elderly_id, lat, lng, geofence_status)

    # Update Redis for real-time access
    await redis.setex(
        f"location:current:{elderly_id}",
        300,  # 5 minute cache
        json.dumps({
            "latitude": lat,
            "longitude": lng,
            "accuracy": accuracy,
            "timestamp": timestamp.isoformat(),
            "inside_geofence": not geofence_status['violated']
        })
    )

    # Broadcast to WebSocket subscribers
    await location_manager.broadcast(elderly_id, {
        "latitude": lat,
        "longitude": lng,
        "accuracy": accuracy,
        "timestamp": timestamp.isoformat()
    })

    return {
        "success": True,
        "geofence_status": geofence_status
    }

async def check_geofence_violation(elderly_id: int, lat: float, lng: float) -> dict:
    """
    Check if location is outside geofence using PostGIS
    """
    result = await db.fetch_one(f'''
        SELECT
            g.id,
            g.name,
            g.radius_meters,
            ST_Distance(
                g.center_point::geography,
                ST_SetSRID(ST_MakePoint({lng}, {lat}), 4326)::geography
            ) as distance_meters,
            ST_Distance(
                g.center_point::geography,
                ST_SetSRID(ST_MakePoint({lng}, {lat}), 4326)::geography
            ) > g.radius_meters as is_outside
        FROM geofences g
        WHERE g.elderly_id = {elderly_id}
            AND g.active = true
        ORDER BY g.priority DESC
        LIMIT 1
    ''')

    if not result:
        return {"violated": False, "message": "No active geofence"}

    return {
        "violated": result['is_outside'],
        "geofence_id": result['id'],
        "geofence_name": result['name'],
        "distance_from_center": round(result['distance_meters'], 2),
        "radius": result['radius_meters']
    }

async def trigger_geofence_alert(elderly_id: int, lat: float, lng: float, geofence_status: dict):
    """
    Trigger alerts when elderly leaves safe zone
    """
    # Get elderly profile
    profile = await get_elderly_profile(elderly_id)

    # Create alert
    alert_id = await db.execute(f'''
        INSERT INTO geofence_alerts (
            elderly_id, geofence_id, location,
            distance_from_center, alert_type, status
        )
        VALUES (
            {elderly_id}, {geofence_status['geofence_id']},
            ST_SetSRID(ST_MakePoint({lng}, {lat}), 4326),
            {geofence_status['distance_from_center']},
            'exit', 'active'
        )
        RETURNING id
    ''')

    # Get emergency contacts
    contacts = await get_emergency_contacts(elderly_id)

    # Send alerts via multiple channels
    await asyncio.gather(
        send_push_notification(contacts, profile, lat, lng),
        send_sms_alert(contacts, profile, lat, lng),
        update_dashboard_alert(elderly_id, lat, lng)
    )

    # Log event
    print(f"GEOFENCE ALERT: {profile['name']} has left safe zone")

    return alert_id

async def send_push_notification(contacts, profile, lat, lng):
    """Send push notification to family members"""
    maps_link = f"https://www.google.com/maps?q={lat},{lng}"

    for contact in contacts:
        await sns.publish(
            TopicArn=f"arn:aws:sns:ap-southeast-1:xxx:emergency-{contact['id']}",
            Message=json.dumps({
                "default": f"⚠️ {profile['name']} has left safe zone",
                "GCM": json.dumps({
                    "notification": {
                        "title": "🚨 Geofence Alert",
                        "body": f"{profile['name']} has left the safe zone. Track location now.",
                        "priority": "high",
                        "sound": "alert.mp3"
                    },
                    "data": {
                        "type": "geofence_violation",
                        "elderly_id": profile['id'],
                        "location": {"lat": lat, "lng": lng},
                        "maps_link": maps_link
                    }
                })
            }),
            MessageStructure='json'
        )

@app.get("/api/location/{elderly_id}/current")
async def get_current_location(elderly_id: int):
    """Get most recent location"""
    # Try Redis first (fast)
    cached = await redis.get(f"location:current:{elderly_id}")
    if cached:
        return json.loads(cached)

    # Fall back to database
    result = await db.fetch_one(f'''
        SELECT
            latitude, longitude, accuracy, recorded_at,
            battery_level, source
        FROM location_history
        WHERE elderly_id = {elderly_id}
        ORDER BY recorded_at DESC
        LIMIT 1
    ''')

    return result

@app.get("/api/location/{elderly_id}/history")
async def get_location_history(
    elderly_id: int,
    start_date: str = None,
    end_date: str = None
):
    """
    Get location history with route visualization
    """
    if not start_date:
        start_date = (datetime.utcnow() - timedelta(days=1)).isoformat()
    if not end_date:
        end_date = datetime.utcnow().isoformat()

    # Query with PostGIS line string generation
    result = await db.fetch_all(f'''
        SELECT
            latitude,
            longitude,
            accuracy,
            recorded_at,
            ST_AsGeoJSON(location) as geojson
        FROM location_history
        WHERE elderly_id = {elderly_id}
            AND recorded_at BETWEEN '{start_date}' AND '{end_date}'
        ORDER BY recorded_at ASC
    ''')

    # Calculate statistics
    if len(result) > 1:
        # Calculate total distance traveled
        distance_query = await db.fetch_one(f'''
            SELECT
                SUM(ST_Distance(
                    lag(location::geography) OVER (ORDER BY recorded_at),
                    location::geography
                )) as total_distance_meters
            FROM location_history
            WHERE elderly_id = {elderly_id}
                AND recorded_at BETWEEN '{start_date}' AND '{end_date}'
        ''')

        total_distance = distance_query['total_distance_meters'] or 0
    else:
        total_distance = 0

    return {
        "elderly_id": elderly_id,
        "start_date": start_date,
        "end_date": end_date,
        "locations": result,
        "total_points": len(result),
        "total_distance_meters": round(total_distance, 2),
        "total_distance_km": round(total_distance / 1000, 2)
    }

@app.post("/api/geofence/create")
async def create_geofence(geofence_data: GeofenceCreate):
    """
    Create a new geofence (safe zone)
    """
    elderly_id = geofence_data.elderly_id
    name = geofence_data.name
    center_lat = geofence_data.center_lat
    center_lng = geofence_data.center_lng
    radius_meters = geofence_data.radius_meters

    result = await db.execute(f'''
        INSERT INTO geofences (
            elderly_id, name, center_point, radius_meters, active
        )
        VALUES (
            {elderly_id}, '{name}',
            ST_SetSRID(ST_MakePoint({center_lng}, {center_lat}), 4326),
            {radius_meters}, true
        )
        RETURNING *
    ''')

    return {
        "success": True,
        "geofence": result
    }

# WebSocket for real-time location streaming
@app.websocket("/location/stream/{elderly_id}")
async def location_stream(websocket: WebSocket, elderly_id: int):
    """Stream real-time location updates to frontend"""
    await location_manager.connect(elderly_id, websocket)
    try:
        while True:
            await asyncio.sleep(1)  # Keep connection alive
    except WebSocketDisconnect:
        await location_manager.disconnect(elderly_id, websocket)

class LocationManager:
    def __init__(self):
        self.connections: Dict[int, List[WebSocket]] = {}

    async def connect(self, elderly_id: int, websocket: WebSocket):
        await websocket.accept()
        if elderly_id not in self.connections:
            self.connections[elderly_id] = []
        self.connections[elderly_id].append(websocket)

    async def disconnect(self, elderly_id: int, websocket: WebSocket):
        if elderly_id in self.connections:
            self.connections[elderly_id].remove(websocket)

    async def broadcast(self, elderly_id: int, message: dict):
        if elderly_id in self.connections:
            for connection in self.connections[elderly_id]:
                await connection.send_json(message)

location_manager = LocationManager()
```

## Hardware Integration - GPS Trackers

### 1. Wearable GPS Watch (e.g., AngelSense GPS)
```python
# GPS watch with cellular connectivity
# Sends location every 60 seconds

import requests
from datetime import datetime

def send_gps_location(lat, lng, accuracy, battery):
    """Send GPS coordinates to cloud"""
    payload = {
        "elderly_id": ELDERLY_ID,
        "latitude": lat,
        "longitude": lng,
        "accuracy": accuracy,
        "battery_level": battery,
        "source": "gps_watch",
        "timestamp": datetime.utcnow().isoformat()
    }

    response = requests.post(
        "https://api.eldercare.com/api/location/update",
        json=payload,
        headers={"Authorization": f"Bearer {DEVICE_TOKEN}"}
    )

    print(f"Location sent: {response.status_code}")

# This runs on the GPS device firmware
# Uses 4G/LTE to send location data
```

### 2. Smartphone App (Fallback)
```javascript
// Mobile app for tracking
// React Native implementation

import Geolocation from '@react-native-community/geolocation';
import BackgroundGeolocation from 'react-native-background-geolocation';

// Configure background tracking
BackgroundGeolocation.configure({
    desiredAccuracy: BackgroundGeolocation.HIGH_ACCURACY,
    stationaryRadius: 50,
    distanceFilter: 50, // Send update every 50 meters
    interval: 60000, // Check every 60 seconds
    fastestInterval: 30000,
    activityType: BackgroundGeolocation.ACTIVITY_TYPE_OTHER,
    stopOnTerminate: false,
    startOnBoot: true
});

BackgroundGeolocation.on('location', (location) => {
    sendLocationToServer(location);
});

BackgroundGeolocation.start();

function sendLocationToServer(location) {
    fetch('https://api.eldercare.com/api/location/update', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${authToken}`
        },
        body: JSON.stringify({
            elderly_id: elderlyId,
            latitude: location.coords.latitude,
            longitude: location.coords.longitude,
            accuracy: location.coords.accuracy,
            battery_level: location.battery.level,
            source: 'mobile_app'
        })
    });
}
```

## Database Schema (PostGIS)
```sql
-- Enable PostGIS extension
CREATE EXTENSION IF NOT EXISTS postgis;

CREATE TABLE location_history (
    id SERIAL PRIMARY KEY,
    elderly_id INTEGER REFERENCES elderly_profiles(id),
    location GEOGRAPHY(POINT, 4326) NOT NULL, -- PostGIS geography type
    latitude DECIMAL(10, 7),
    longitude DECIMAL(10, 7),
    accuracy FLOAT, -- meters
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    battery_level INTEGER, -- percentage
    source VARCHAR(50) -- gps_watch, mobile_app, smart_watch
);

CREATE TABLE geofences (
    id SERIAL PRIMARY KEY,
    elderly_id INTEGER REFERENCES elderly_profiles(id),
    name VARCHAR(255), -- 'Home', 'Park', 'Community Center'
    center_point GEOGRAPHY(POINT, 4326) NOT NULL,
    radius_meters INTEGER NOT NULL,
    active BOOLEAN DEFAULT true,
    priority INTEGER DEFAULT 1, -- For overlapping geofences
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE geofence_alerts (
    id SERIAL PRIMARY KEY,
    elderly_id INTEGER REFERENCES elderly_profiles(id),
    geofence_id INTEGER REFERENCES geofences(id),
    location GEOGRAPHY(POINT, 4326),
    distance_from_center FLOAT, -- meters
    alert_type VARCHAR(20), -- 'exit', 'enter'
    status VARCHAR(20) DEFAULT 'active', -- active, acknowledged, resolved
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    acknowledged_at TIMESTAMP
);

-- Spatial indexes for performance
CREATE INDEX idx_location_history_spatial ON location_history USING GIST(location);
CREATE INDEX idx_location_history_elderly_time ON location_history(elderly_id, recorded_at DESC);
CREATE INDEX idx_geofences_spatial ON geofences USING GIST(center_point);
CREATE INDEX idx_geofences_elderly ON geofences(elderly_id) WHERE active = true;
```

## Tech Stack
- **Backend:** Python 3.11 + FastAPI
- **Database:** PostgreSQL 15 + PostGIS 3.3
- **GIS:** PostGIS for spatial queries
- **Real-time:** WebSocket
- **Maps:** Google Maps API
- **GPS Hardware:** AngelSense GPS Tracker / Apple AirTag / Samsung SmartTag
- **Mobile:** React Native (iOS/Android)
- **Notifications:** AWS SNS, Twilio SMS

## AI/ML Component - Location Pattern Analysis

```python
# Detect unusual movement patterns (dementia wandering detection)
from sklearn.cluster import DBSCAN
import numpy as np

class LocationPatternAnalyzer:
    def analyze_daily_pattern(self, location_history):
        """
        Analyze if today's movement is unusual compared to historical pattern
        """
        # Get coordinates
        coords = np.array([[loc['latitude'], loc['longitude']]
                          for loc in location_history])

        # Cluster analysis using DBSCAN
        clustering = DBSCAN(eps=0.001, min_samples=5).fit(coords)

        # Identify usual locations (clusters)
        usual_locations = []
        for cluster_id in set(clustering.labels_):
            if cluster_id == -1:  # Noise
                continue

            cluster_points = coords[clustering.labels_ == cluster_id]
            center = cluster_points.mean(axis=0)
            usual_locations.append({
                "lat": center[0],
                "lng": center[1],
                "visit_count": len(cluster_points)
            })

        return usual_locations

    def detect_wandering(self, recent_locations, usual_locations):
        """
        Detect if elderly is wandering (away from usual places)
        """
        if not recent_locations:
            return False

        latest = recent_locations[-1]
        latest_point = (latest['latitude'], latest['longitude'])

        # Check distance to all usual locations
        min_distance = float('inf')
        for usual in usual_locations:
            dist = self.haversine_distance(
                latest_point,
                (usual['lat'], usual['lng'])
            )
            min_distance = min(min_distance, dist)

        # If more than 500m from any usual location
        if min_distance > 0.5:  # 500 meters
            return True

        return False

    def haversine_distance(self, point1, point2):
        """Calculate distance in km between two lat/lng points"""
        from math import radians, sin, cos, sqrt, atan2

        lat1, lon1 = map(radians, point1)
        lat2, lon2 = map(radians, point2)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))

        R = 6371  # Earth radius in km
        return R * c
```

## Scope
- **Basic:** Manual location check, static geofence
- **Advanced:** Real-time tracking, automated alerts, pattern analysis
- **Future:** Predictive wandering detection, breadcrumb trails, integration with emergency services GPS, indoor positioning (BLE beacons)

---

*[Document continues with remaining features: Wearable Devices Integration, Environmental Monitoring, Voice Control Assistant, Smart Automation Rules Engine, Complete System Integration, Deployment Architecture, Security & Compliance]*

---

# Summary Tech Stack Table (Complete System)

| Component | Technology | Purpose | Cost (Monthly) |
|-----------|------------|---------|----------------|
| Frontend | React 18 + TypeScript | Web dashboard | Free (hosted) |
| Mobile App | React Native | iOS/Android apps | Free |
| API Gateway | Kong OSS | API routing, auth | RM 150 |
| Auth Service | Node.js + JWT | Authentication | RM 50 |
| Profile Service | Node.js + Express | User data | RM 80 |
| Health Service | Python + FastAPI | Vitals, AI | RM 200 |
| IoT Service | Node.js + MQTT | Device control | RM 120 |
| Location Service | Python + PostGIS | GPS tracking | RM 100 |
| Notification Service | Node.js + SNS | Push, SMS, email | RM 150 |
| Main Database | PostgreSQL 15 RDS | Relational data | RM 400 |
| Time-Series DB | InfluxDB Cloud | Vitals data | RM 180 |
| Cache/Queue | Redis ElastiCache | Real-time state | RM 120 |
| File Storage | AWS S3 | Images, videos | RM 80 |
| ML Training | AWS SageMaker | Model training | RM 200 |
| ML Serving | TensorFlow Serving | Inference | RM 100 |
| Message Broker | MQTT (Mosquitto) | IoT messages | RM 60 |
| **TOTAL** | | **Per 100 patients** | **~RM 1,990** |

---

**End of Part 2**
