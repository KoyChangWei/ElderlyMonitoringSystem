# ElderCare AI Monitoring System Dashboard

## Overview
A comprehensive dashboard for monitoring and managing care for 4 elderly residents with AI-powered insights, real-time health monitoring, and automated assistance.

## Project Structure
```
latest_asg2/
├── index.html              # Main overview dashboard
├── css/
│   └── styles.css          # All styles with Gestalt principles
├── js/
│   └── app.js              # Data management & interactivity
└── pages/
    ├── health-analytics.html   # Health monitoring & AI predictions
    ├── robot-control.html      # Robot caregiver control center
    ├── emergency.html          # Safety & emergency management
    ├── smart-home.html         # IoT device controls
    ├── medication.html         # Medication management system
    └── user-management.html    # Manage residents & wristband assignments
```

## Features

### 1. Main Dashboard (index.html)
- **Multi-Elderly Sidebar**: 4 color-coded residents with real-time status
- **Critical Alerts Banner**: Dismissible color-coded alerts
- **AI Health Analytics Table**: Live vitals with AI predictions
- **Robot Scheduling Preview**: Upcoming robot tasks
- **Safety & Activity Monitoring**: Steps, sleep, falls, location
- **Health Trends Chart**: 24-hour interactive Chart.js visualization
- **Smart Home IoT Controls**: Quick toggle switches
- **Today's Medication Schedule**: Color-coded by resident
- **Emergency Contacts**: Quick dial buttons
- **Real-time Activity Feed**: Live updates

### 2. Health Analytics Page
- **Resident Selector**: Switch between residents
- **Current Vitals Display**: Large metric cards
- **4 Trend Charts**: Heart Rate, Blood Pressure, Blood Sugar, Temperature
- **AI Health Predictions**: Stability scores with recommendations
- **Weekly Health Summary**: Comprehensive health reports

### 3. Robot Control Page
- **Robot Camera Feed**: Simulated live view with HUD overlay
- **Movement Controls**: Directional pad for robot navigation
- **Quick Actions**: 6 action buttons (Speak, Video Call, Medicine, etc.)
- **Operation Modes**: Companion, Assistant, Security, Emergency
- **Task Scheduler**: Add/edit/delete scheduled robot tasks
- **Recent Activities**: Robot activity history

### 4. Safety & Emergency Page
- **Emergency SOS Button**: Large, prominent emergency activation
- **AI Fall Detection**: Status from cameras, shoes, and pendant
- **Location Tracking**: GPS with geofencing (500m safe zone)
- **Smart Bed Monitoring**: 4 metrics + 4 sensor cards
- **Wearable Sensors**: Smart shoes and emergency pendant
- **AI Camera Status**: 4 active cameras with AI features
- **Emergency Contacts**: Quick call/video options
- **Safety Alert History**: Recent incidents and resolution

### 5. Smart Home IoT Page
- **5 Device Cards**: Lights, AC, Cameras, Door Lock, Voice Assistant
- **Interactive Controls**: Toggle switches, sliders for brightness/temp
- **Automation Rules**: 4 pre-configured rules
- **Energy Usage**: 24-hour bar chart + device breakdown

### 6. Medication Page
- **Quick Actions**: Add medication, mark taken, set reminder, order refill
- **Today's Timeline**: Visual timeline with status markers
- **Weekly Compliance**: Doughnut chart with statistics
- **Active Medications**: Detailed cards for all residents
- **Refill Alerts**: Warning for low supply
- **Compliance Calendar**: 7-day tracking table

### 7. User Management Page ⭐ NEW
- **Add New Residents**: Form with personal info, age, room assignment
- **Wristband Assignment**: Auto-generated unique wristband IDs (WB-001, WB-002, etc.)
- **Color Selection**: 12 color options for visual identification
- **Edit Residents**: Update existing resident information
- **Delete Residents**: Remove residents with confirmation
- **Statistics Dashboard**: Total residents, active wristbands, health status
- **Medical Conditions**: Track and manage health conditions per resident
- **LocalStorage Integration**: Persistent data across all pages
- **Resident Cards**: Detailed view with vitals, room, and conditions

## Design Principles

### Gestalt Principles Applied
1. **Proximity**: Related elements grouped together (vitals, residents, alerts)
2. **Similarity**: Consistent styling for similar elements (cards, buttons, badges)
3. **Closure**: Complete visual patterns (cards, borders, containers)
4. **Continuity**: Visual flow from top to bottom, left to right
5. **Figure-Ground**: Clear separation using shadows, borders, colors

### PowerBI-Style Layout
- **Compact Grid System**: Minimal empty space
- **Responsive Cards**: Auto-fit grid layout
- **Dense Information Display**: Maximum data in minimum space
- **Color Coding**: 4 resident colors throughout all pages

## Color Scheme

### Residents
- 🔵 **Ah Chong Tan**: Blue (#3B82F6)
- 🟣 **Mei Ling Wong**: Purple (#8B5CF6)
- 🟢 **Kumar Raj**: Teal (#14B8A6)
- 🟠 **Fatimah Ibrahim**: Orange (#F59E0B)

### System Colors
- **Background**: Dark blue-gray (#0f172a, #1e293b)
- **Status Normal**: Green (#10b981)
- **Status Warning**: Orange (#f59e0b)
- **Status Critical**: Red (#ef4444)
- **Status Info**: Blue (#3b82f6)

## Data Structure

All elderly data is managed in `js/app.js`:
```javascript
elderlyList = [
  {
    id: 1,
    personal: { name, age, color, wristbandId, conditions },
    vitals: { heartRate, bloodPressure, bloodSugar, temperature },
    activity: { steps, sleep, location, aiPrediction },
    alerts: []
  }
]
```

## Real-time Updates
- **Vital Signs**: Updates every 5 seconds
- **Current Time**: Updates every 1 second
- **Activity Feed**: New item every 30 seconds
- **Chart Data**: Live chart updates on metric change

## How to Use

### Opening the Dashboard
1. Open `index.html` in a web browser
2. All pages are linked via the sidebar navigation
3. No server required - pure HTML/CSS/JavaScript

### Navigation
- **Sidebar**: Click any menu item to navigate to different pages
- **Cards**: Click on cards in overview to go to detailed pages
- **Resident Selection**: Click resident cards or buttons to filter data

### Interactive Elements
- **Toggle Switches**: Click to turn devices on/off
- **Sliders**: Drag to adjust brightness/temperature
- **Charts**: Hover for tooltips, use dropdown to switch metrics
- **Buttons**: Click for actions (alerts will show simulated responses)

## Browser Compatibility
- **Recommended**: Chrome, Edge, Firefox (latest versions)
- **Required**: JavaScript enabled
- **CDN Dependencies**:
  - Chart.js (for visualizations)
  - Font Awesome (for icons)

## Future Enhancements (Not Implemented - Prototype Only)
- Real backend API integration
- Live camera feeds
- Actual robot control via MQTT
- Database for persistent storage
- User authentication
- Mobile app version
- Push notifications
- Voice commands integration

## Notes
- This is a **PROTOTYPE** using raw/simulated data
- All data is hardcoded in JavaScript
- No real IoT device connections
- Simulated real-time updates for demonstration
- Alert actions show confirmation dialogs (not real functionality)

## Developer Information
- **Framework**: Vanilla JavaScript (no frameworks)
- **Styling**: Custom CSS with CSS Grid and Flexbox
- **Charts**: Chart.js library
- **Icons**: Font Awesome 6.4.0
- **Responsive**: Mobile-friendly with media queries

## Support
For questions or issues, refer to the code comments in each file.

---
**ElderCare AI Monitoring System** - Comprehensive elderly care management dashboard
