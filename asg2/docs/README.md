# Elderly Care Monitoring Dashboard

## Overview
An innovative, AI-powered dashboard prototype for monitoring and caring for elderly individuals. This system integrates cutting-edge technologies including robot caregivers, IoT smart home devices, AI health analytics, and predictive monitoring systems.

## Assignment Context
- **Course**: STTHK3133 Human Factor Engineering
- **Semester**: 1 Session 2025/2026 (A251)
- **Assignment**: 2 - Cognition, Memory, and Training
- **Project Type**: Elderly Care Monitoring Dashboard (Option 7)

## Key Features

### 1. **AI-Powered Health Monitoring**
- Real-time vital signs tracking (Heart Rate, Blood Pressure, O2 Saturation, Temperature)
- Predictive health analytics using machine learning
- 95% health stability prediction based on 30-day analysis
- Connected wearable devices (smartwatch, smart shoes, smart ring, smart bed)
- Automated health trend analysis with visual charts

### 2. **Robot Caregiver Control**
- Interactive robot control interface with live camera feed
- Multiple operation modes: Companion, Assistant, Security, Emergency
- Movement controls (forward, backward, left, right, stop)
- Quick actions: speak, video call, medicine reminder, patrol, music, fetch items
- AI conversation capabilities for social interaction
- Fall detection and emergency response
- Battery monitoring and status tracking

### 3. **Smart Home IoT Integration**
- Voice control assistant
- Environmental monitoring (temperature, humidity, air quality, light levels)
- Smart devices:
  - Intelligent lighting with brightness and color temperature control
  - Air conditioning with temperature and mode settings
  - Smart door locks with auto-lock scheduling
  - AI security cameras (4 cameras across different rooms)
  - Smart bed sensors for sleep monitoring
  - Smart toilet sensors for health tracking
- Automated rules for bedtime, morning routines, fall response, temperature comfort

### 4. **Emergency & Safety Systems**
- Large, accessible SOS emergency button
- GPS location tracking with geofencing
- AI fall detection using computer vision
- Multi-layered safety monitoring:
  - Computer vision behavior analysis
  - Smoke and fire detection
  - Water leak detection
  - Inactivity alert system
  - Door and window sensors
- Quick dial emergency contacts
- Real-time alert history

### 5. **Medication & Nutrition Management**
- AI smart medication dispenser
- Visual timeline for daily medication schedule
- 98% medication compliance tracking
- Automatic reminders via robot and notifications
- Detailed medication profiles with dosage information
- Nutrition tracking with meal logging
- Daily calorie, protein, and nutrient goals
- Hydration tracker (8 glasses daily goal)
- Smart fridge monitoring with expiry warnings
- 7-day medication adherence charts

## Human Factors Engineering Principles Applied

### Gestalt Principles
1. **Proximity**: Related elements grouped together (vital signs, device controls, contact information)
2. **Similarity**: Consistent styling for similar functions (all IoT devices, all status indicators)
3. **Continuity**: Visual flow through aligned elements and clear navigation paths
4. **Closure**: Complete visual patterns in cards and sections
5. **Figure-Ground**: Clear distinction between elements and backgrounds using shadows and colors

### Cognitive Load Reduction
- Clear visual hierarchy with large, readable fonts (base 16px)
- Color-coded status indicators (green=normal, yellow=warning, red=critical)
- Consistent icon usage across the interface
- Minimal text with focus on visual information
- Progressive disclosure (overview → detailed pages)

### Memory Support
- Visual reminders and notifications
- Timeline views for medication schedules
- Status badges and indicators
- Recent activity feeds
- Color associations for different functions

### Accessibility for Elderly Users
- Large click targets (minimum 44px height)
- High contrast colors
- Clear, simple language
- Intuitive icons with text labels
- Touch-friendly interface
- Emergency SOS button prominently displayed
- Voice control option for hands-free operation

## Technology Stack
- **HTML5**: Structure and semantic markup
- **CSS3**: Styling with CSS Grid, Flexbox, gradients, animations
- **JavaScript (ES6)**: Interactive functionality and real-time updates
- **Chart.js**: Data visualization for health trends and adherence
- **Font Awesome**: Icon library for intuitive visual communication

## File Structure
```
asg2/
├── index.html              # Main dashboard page
├── health-analytics.html   # Detailed health monitoring page
├── robot-control.html      # Robot caregiver control interface
├── smart-home.html         # IoT smart home control page
├── emergency.html          # Emergency and safety management
├── medication.html         # Medication and nutrition tracking
├── styles.css              # Comprehensive styling with Gestalt principles
├── script.js               # Main JavaScript with real-time updates
└── README.md              # This file
```

## How to Run
1. Open `index.html` in a modern web browser (Chrome, Firefox, Edge, Safari)
2. No server or installation required - runs entirely in the browser
3. Navigate between pages using the sidebar menu or clicking on dashboard cards
4. All data is simulated with realistic values for demonstration purposes

## Innovative Features Not Yet Common in Malaysia

### 1. **Robot Caregiver Integration**
- Physical robot that can move, fetch items, and provide companionship
- AI-powered conversation for combating loneliness
- Remote control via dashboard interface

### 2. **Predictive AI Health Analytics**
- Machine learning predicting health issues before they occur
- 95% stability score based on pattern analysis
- Early warning system for deteriorating health

### 3. **Comprehensive Wearable Ecosystem**
- Smart shoes with gait analysis and fall detection
- Smart ring for continuous temperature monitoring
- Smart bed sensors for sleep quality analysis
- Integration with smartwatch for comprehensive data

### 4. **Smart Toilet Health Monitoring**
- Non-invasive health analysis through toilet sensors
- Hydration level detection
- Automated health tracking

### 5. **AI Computer Vision Fall Detection**
- Real-time analysis of movement patterns
- Instant alerts on falls or near-falls
- Privacy-focused with on-device processing

### 6. **Smart Medication Dispenser**
- Automated pill dispensing at scheduled times
- Robot delivery to the person
- Compliance tracking and family notifications

### 7. **Geofencing & GPS Tracking**
- Safe zone monitoring for wandering prevention
- Instant alerts if elderly leaves designated area
- Useful for dementia patients

### 8. **Voice-Controlled Smart Home**
- Senior-friendly voice commands
- Integration with all IoT devices
- Reduces need for physical interaction

### 9. **Smart Fridge Nutrition Management**
- Automatic food inventory tracking
- Expiry date monitoring
- Nutritional analysis and meal suggestions
- Shopping list generation

### 10. **Integrated Emergency Response System**
- One-touch SOS activates multiple protocols
- Automatic door unlocking for first responders
- GPS location sharing with emergency services
- Family and caregiver instant notifications

## Design Philosophy

### User-Centered Design
- Designed for caregivers, family members, and healthcare professionals
- Large, clear text suitable for all age groups
- Intuitive navigation requiring minimal training
- Visual feedback for all interactions

### Safety-First Approach
- Emergency features prominently displayed
- Multiple layers of safety monitoring
- Redundant alert systems
- Quick access to emergency contacts

### Data-Driven Insights
- Real-time monitoring and updates
- Historical trend analysis
- Predictive analytics for proactive care
- Comprehensive reporting for healthcare providers

## Future Enhancements
- Integration with actual IoT devices via APIs
- Machine learning model training with real patient data
- Mobile app for family members
- Multi-language support (English, Malay, Chinese, Tamil)
- Electronic health records (EHR) integration
- Telemedicine video consultation
- Social engagement features (virtual activities, games)
- Cognitive training exercises for dementia prevention

## Browser Compatibility
- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Credits
Developed for STTHK3133 Human Factor Engineering Assignment 2
Universiti Utara Malaysia (UUM)

## Notes
- All data displayed is simulated for demonstration purposes
- Icons and avatars are generated using external CDN services
- Charts update with random realistic data to simulate real-time monitoring
- No backend or database required - fully client-side application

## License
Educational use only - UUM STTHK3133 Assignment
