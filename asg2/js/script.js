// ===================================
// Elderly Care Monitoring Dashboard
// Main JavaScript with Real-time Updates
// ===================================

// Global Variables
let healthChart = null;
let currentChartType = 'heartRate';

// ===================================
// INITIALIZATION
// ===================================
document.addEventListener('DOMContentLoaded', function() {
    initializeClock();
    initializeHealthChart();
    startRealTimeUpdates();
    initializeNotifications();
    
    // For prototype: Messages are static in HTML, just update mental health UI
    if (document.getElementById('chatMessages')) {
        updateMentalHealthUI();
        simulateMentalHealthUpdates();
        
        // Update last chat time for display
        const lastChatTime = document.getElementById('lastChatTime');
        if (lastChatTime) {
            lastChatTime.textContent = 'Last chat: Just now';
        }
    }
});

// ===================================
// CLOCK UPDATE
// ===================================
function initializeClock() {
    updateClock();
    setInterval(updateClock, 1000);
}

function updateClock() {
    const now = new Date();
    const options = {
        weekday: 'short',
        year: 'numeric',
        month: 'short',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    };
    document.getElementById('currentTime').textContent = now.toLocaleDateString('en-MY', options);
}

// ===================================
// HEALTH CHART - Using Chart.js
// ===================================
function initializeHealthChart() {
    const ctx = document.getElementById('healthChart');
    if (!ctx) return;

    const data = {
        labels: generateTimeLabels(24),
        datasets: [{
            label: 'Heart Rate (bpm)',
            data: generateHeartRateData(),
            borderColor: '#E74C3C',
            backgroundColor: 'rgba(231, 76, 60, 0.1)',
            borderWidth: 3,
            tension: 0.4,
            fill: true,
            pointRadius: 4,
            pointBackgroundColor: '#E74C3C'
        }]
    };

    const config = {
        type: 'line',
        data: data,
        options: {
            responsive: true,
            maintainAspectRatio: true,
            plugins: {
                legend: {
                    display: true,
                    labels: {
                        font: {
                            size: 14
                        }
                    }
                },
                tooltip: {
                    mode: 'index',
                    intersect: false,
                    backgroundColor: 'rgba(0, 0, 0, 0.8)',
                    padding: 12,
                    titleFont: {
                        size: 14
                    },
                    bodyFont: {
                        size: 13
                    }
                }
            },
            scales: {
                y: {
                    beginAtZero: false,
                    grid: {
                        color: 'rgba(0, 0, 0, 0.05)'
                    },
                    ticks: {
                        font: {
                            size: 12
                        }
                    }
                },
                x: {
                    grid: {
                        display: false
                    },
                    ticks: {
                        font: {
                            size: 12
                        }
                    }
                }
            },
            interaction: {
                mode: 'nearest',
                axis: 'x',
                intersect: false
            }
        }
    };

    healthChart = new Chart(ctx, config);
}

// ===================================
// CHART DATA GENERATORS
// ===================================
function generateTimeLabels(hours) {
    const labels = [];
    const now = new Date();
    for (let i = hours - 1; i >= 0; i--) {
        const time = new Date(now.getTime() - i * 60 * 60 * 1000);
        labels.push(time.getHours() + ':00');
    }
    return labels;
}

function generateHeartRateData() {
    const data = [];
    const baseRate = 72;
    for (let i = 0; i < 24; i++) {
        // Simulate realistic heart rate variation
        const variation = Math.sin(i / 3) * 8 + (Math.random() - 0.5) * 6;
        data.push(Math.round(baseRate + variation));
    }
    return data;
}

function generateBloodPressureData() {
    const data = [];
    const baseSystolic = 120;
    for (let i = 0; i < 24; i++) {
        const variation = Math.sin(i / 4) * 10 + (Math.random() - 0.5) * 8;
        data.push(Math.round(baseSystolic + variation));
    }
    return data;
}

function generateBloodSugarData() {
    const data = [];
    const baseBloodSugar = 5.8; // mmol/L - normal fasting range
    for (let i = 0; i < 24; i++) {
        const variation = (Math.random() - 0.5) * 0.8; // smaller variation for blood sugar
        data.push(Math.round((baseBloodSugar + variation) * 10) / 10);
    }
    return data;
}

function generateTemperatureData() {
    const data = [];
    const baseTemp = 36.8;
    for (let i = 0; i < 24; i++) {
        const variation = Math.sin(i / 5) * 0.4 + (Math.random() - 0.5) * 0.3;
        data.push(Math.round((baseTemp + variation) * 10) / 10);
    }
    return data;
}

// ===================================
// CHART UPDATE FUNCTION
// ===================================
function updateChart() {
    if (!healthChart) return;

    const selector = document.getElementById('chartSelector');
    currentChartType = selector.value;

    let newData, label, color, bgColor;

    switch(currentChartType) {
        case 'heartRate':
            newData = generateHeartRateData();
            label = 'Heart Rate (bpm)';
            color = '#E74C3C';
            bgColor = 'rgba(231, 76, 60, 0.1)';
            break;
        case 'bloodPressure':
            newData = generateBloodPressureData();
            label = 'Blood Pressure (Systolic)';
            color = '#9B59B6';
            bgColor = 'rgba(155, 89, 182, 0.1)';
            break;
        case 'bloodSugar':
            newData = generateBloodSugarData();
            label = 'Blood Sugar (mmol/L)';
            color = '#94C9A9';
            bgColor = 'rgba(148, 201, 169, 0.1)';
            break;
        case 'temperature':
            newData = generateTemperatureData();
            label = 'Temperature (°C)';
            color = '#F39C12';
            bgColor = 'rgba(243, 156, 18, 0.1)';
            break;
    }

    healthChart.data.datasets[0].data = newData;
    healthChart.data.datasets[0].label = label;
    healthChart.data.datasets[0].borderColor = color;
    healthChart.data.datasets[0].backgroundColor = bgColor;
    healthChart.data.datasets[0].pointBackgroundColor = color;
    healthChart.update();
}

// ===================================
// REAL-TIME VITAL SIGNS UPDATE
// ===================================
function startRealTimeUpdates() {
    // Update vital signs every 5 seconds
    setInterval(updateVitalSigns, 5000);

    // Update robot battery every 30 seconds
    setInterval(updateRobotBattery, 30000);

    // Update activity status every 10 seconds
    setInterval(updateActivityStatus, 10000);
}

function updateVitalSigns() {
    // Simulate real-time vital signs with slight variations
    const heartRate = document.getElementById('heartRate');
    const currentHR = parseInt(heartRate.textContent);
    const newHR = currentHR + (Math.random() - 0.5) * 4;
    heartRate.textContent = Math.round(newHR) + ' bpm';

    // Update blood pressure
    const bp = document.getElementById('bloodPressure');
    const variation = Math.random() > 0.5 ? 1 : -1;
    const currentSystolic = parseInt(bp.textContent.split('/')[0]);
    const currentDiastolic = parseInt(bp.textContent.split('/')[1]);
    bp.textContent = `${currentSystolic + variation}/${currentDiastolic}`;

    // Update blood sugar (stays relatively stable)
    const bloodSugar = document.getElementById('bloodSugar');
    if (bloodSugar) {
        const currentBS = parseFloat(bloodSugar.textContent);
        const newBS = currentBS + (Math.random() - 0.5) * 0.2;
        bloodSugar.textContent = newBS.toFixed(1) + ' mmol/L';
    }

    // Update temperature (minimal variation)
    const temp = document.getElementById('temperature');
    const currentTemp = parseFloat(temp.textContent);
    const newTemp = currentTemp + (Math.random() - 0.5) * 0.1;
    temp.textContent = newTemp.toFixed(1) + '°C';
}

function updateRobotBattery() {
    const battery = document.getElementById('robotBattery');
    const currentBattery = parseInt(battery.textContent);
    // Decrease by 1% occasionally
    if (Math.random() > 0.7 && currentBattery > 20) {
        battery.textContent = (currentBattery - 1) + '%';
    }
}

function updateActivityStatus() {
    // Could update step count, mood, etc.
    // For demo, we'll keep it static but this could pull real data
}

// ===================================
// DEVICE CONTROL
// ===================================
function toggleDevice(element, deviceType) {
    const isOn = element.checked;
    const status = isOn ? 'ON' : 'OFF';

    // Show notification
    showNotification(`${deviceType.toUpperCase()} turned ${status}`, 'info');

    // In real application, this would send command to IoT device
    console.log(`${deviceType} is now ${status}`);
}

// ===================================
// NAVIGATION
// ===================================
function navigateTo(page) {
    window.location.href = page;
}

// ===================================
// EMERGENCY FUNCTIONS
// ===================================
function triggerEmergency() {
    const confirmation = confirm('EMERGENCY ALERT\n\nThis will:\n- Call emergency services (999)\n- Alert all family members\n- Activate all cameras\n- Send location to responders\n\nProceed?');

    if (confirmation) {
        // Change alert status
        const alertStatus = document.getElementById('alertStatus');
        alertStatus.innerHTML = '<i class="fas fa-exclamation-triangle"></i><span>EMERGENCY ACTIVATED</span>';
        alertStatus.style.background = 'rgba(231, 76, 60, 0.3)';

        // Show emergency banner
        showEmergencyBanner();

        // In real application:
        // - Call emergency API
        // - Send SMS/notifications
        // - Activate emergency protocols

        alert('Emergency services have been notified!\nHelp is on the way.');
    }
}

function showEmergencyBanner() {
    const banner = document.getElementById('alertsBanner');
    const emergencyAlert = document.createElement('div');
    emergencyAlert.className = 'alert-item';
    emergencyAlert.style.background = '#F8D7DA';
    emergencyAlert.style.color = '#721C24';
    emergencyAlert.style.borderLeft = '4px solid #E74C3C';
    emergencyAlert.innerHTML = `
        <i class="fas fa-ambulance"></i>
        <span><strong>EMERGENCY ALERT:</strong> Emergency services have been contacted. Help is on the way.</span>
    `;
    banner.insertBefore(emergencyAlert, banner.firstChild);
}

// ===================================
// NOTIFICATIONS SYSTEM
// ===================================
function initializeNotifications() {
    // Check medication reminders
    checkMedicationReminders();
    setInterval(checkMedicationReminders, 60000); // Check every minute
}

function checkMedicationReminders() {
    const now = new Date();
    const currentTime = now.getHours() * 60 + now.getMinutes();

    // Medication times (in minutes from midnight)
    const medTimes = [
        { time: 8 * 60, name: 'Blood Pressure Medicine', reminded: false },
        { time: 12 * 60, name: 'Diabetes Medication', reminded: false },
        { time: 14 * 60, name: 'Blood Pressure Medicine', reminded: false },
        { time: 20 * 60, name: 'Cholesterol Medication', reminded: false }
    ];

    medTimes.forEach(med => {
        // Remind 15 minutes before
        if (currentTime === med.time - 15 && !med.reminded) {
            showNotification(`Medication due in 15 minutes: ${med.name}`, 'warning');
            med.reminded = true;
        }
    });
}

function showNotification(message, type = 'info') {
    // Create notification element
    const notification = document.createElement('div');
    notification.className = `notification notification-${type}`;
    notification.style.cssText = `
        position: fixed;
        top: 120px;
        right: 30px;
        background: ${type === 'warning' ? '#FFF3CD' : type === 'info' ? '#D1ECF1' : '#D4EDDA'};
        color: ${type === 'warning' ? '#856404' : type === 'info' ? '#0C5460' : '#155724'};
        padding: 16px 24px;
        border-radius: 8px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        z-index: 10000;
        min-width: 300px;
        animation: slideIn 0.3s ease;
        border-left: 4px solid ${type === 'warning' ? '#F39C12' : type === 'info' ? '#3498DB' : '#27AE60'};
    `;
    notification.textContent = message;

    document.body.appendChild(notification);

    // Auto remove after 5 seconds
    setTimeout(() => {
        notification.style.animation = 'slideOut 0.3s ease';
        setTimeout(() => {
            notification.remove();
        }, 300);
    }, 5000);
}

// ===================================
// DATA SIMULATION & MOCK API
// ===================================
const elderlyData = {
    personal: {
        name: 'Ah Chong Tan',
        age: 78,
        room: 'A-203',
        conditions: ['Hypertension', 'Type 2 Diabetes', 'Mild Arthritis']
    },
    vitals: {
        heartRate: { current: 72, normal: [60, 100], unit: 'bpm' },
        bloodPressure: { systolic: 120, diastolic: 80, unit: 'mmHg' },
        bloodSugar: { current: 5.8, normal: [4.0, 6.0], unit: 'mmol/L' },
        temperature: { current: 36.8, normal: [36.1, 37.2], unit: '°C' }
    },
    medications: [
        { name: 'Amlodipine', dosage: '5mg', frequency: '2x daily', times: ['08:00', '14:00'] },
        { name: 'Metformin', dosage: '500mg', frequency: '2x daily', times: ['12:00', '20:00'] },
        { name: 'Atorvastatin', dosage: '20mg', frequency: '1x daily', times: ['20:00'] }
    ],
    activity: {
        steps: 2847,
        stepGoal: 5000,
        sleepHours: 7.5,
        sleepQuality: 'Good',
        lastFall: null,
        moodScore: 85
    },
    robot: {
        status: 'Active',
        battery: 87,
        location: 'Living Room',
        currentTask: 'Companion Mode',
        capabilities: ['Conversation', 'Medication Reminder', 'Fall Detection', 'Video Call']
    },
    iotDevices: {
        lights: { status: true, location: 'Living Room', type: 'Smart LED' },
        ac: { status: true, temperature: 24, mode: 'Auto' },
        camera: { status: true, count: 4, recording: true },
        doorLock: { status: true, locked: true },
        airQuality: { aqi: 45, status: 'Good' },
        voiceAssistant: { status: true, listening: true }
    },
    emergencyContacts: [
        { name: 'Dr. Ahmad Hassan', role: 'Primary Physician', phone: '+60123456789' },
        { name: 'Siti Tan', role: 'Daughter', phone: '+60198765432' },
        { name: 'Emergency 999', role: 'Ambulance Service', phone: '999' },
        { name: 'Nurse Maya', role: 'Home Care Nurse', phone: '+60187654321' }
    ]
};

// ===================================
// UTILITY FUNCTIONS
// ===================================
function getVitalStatus(vital, value) {
    if (!vital.normal) return 'normal';
    const [min, max] = vital.normal;
    if (value < min || value > max) return 'critical';
    if (value < min * 1.1 || value > max * 0.9) return 'warning';
    return 'normal';
}

function formatTime(date) {
    return date.toLocaleTimeString('en-MY', { hour: '2-digit', minute: '2-digit' });
}

function formatDate(date) {
    return date.toLocaleDateString('en-MY', {
        weekday: 'short',
        year: 'numeric',
        month: 'short',
        day: 'numeric'
    });
}

// ===================================
// VOICE CONTROL SIMULATION
// ===================================
function activateVoiceControl() {
    showNotification('Voice Assistant activated. Listening...', 'info');
    // In real application, integrate with Web Speech API
}

// ===================================
// AI MENTAL HEALTH MONITORING
// ===================================

// Mental Health Data Structure with Detailed Tracking
const mentalHealthData = {
    currentState: {
        happiness: 85,
        anxiety: 15,
        sadness: 10,
        engagement: 80
    },
    chatHistory: [
        { 
            sender: 'robot', 
            message: 'Hello Ah Chong! How are you today?', 
            time: '2:15 PM',
            fullTime: '2025-11-25 14:15:32',
            timestamp: new Date('2025-11-25T14:15:32').getTime(),
            sentiment: 'neutral',
            emotionType: null
        },
        { 
            sender: 'elderly', 
            message: 'I am good, thank you! I wanna eat something.', 
            time: '2:16 PM',
            fullTime: '2025-11-25 14:16:15',
            timestamp: new Date('2025-11-25T14:16:15').getTime(),
            sentiment: 'positive',
            emotionType: 'positive'
        },
        { 
            sender: 'robot', 
            message: 'That\'s great! What would you like to eat? Are you feeling hungry now?', 
            time: '2:16 PM',
            fullTime: '2025-11-25 14:16:48',
            timestamp: new Date('2025-11-25T14:16:48').getTime(),
            sentiment: 'neutral',
            emotionType: null
        },
        { 
            sender: 'elderly', 
            message: 'Yes, I feel a bit hungry. Maybe some noodles or rice would be nice.', 
            time: '2:17 PM',
            fullTime: '2025-11-25 14:17:22',
            timestamp: new Date('2025-11-25T14:17:22').getTime(),
            sentiment: 'positive',
            emotionType: 'positive'
        },
        { 
            sender: 'robot', 
            message: 'Noodles sound delicious! Have you had your lunch already, or is this for a snack?', 
            time: '2:18 PM',
            fullTime: '2025-11-25 14:18:05',
            timestamp: new Date('2025-11-25T14:18:05').getTime(),
            sentiment: 'neutral',
            emotionType: null
        },
        { 
            sender: 'elderly', 
            message: 'I had lunch earlier, but I feel like having something light now. Maybe some soup noodles.', 
            time: '2:19 PM',
            fullTime: '2025-11-25 14:19:11',
            timestamp: new Date('2025-11-25T14:19:11').getTime(),
            sentiment: 'positive',
            emotionType: 'positive'
        },
        { 
            sender: 'robot', 
            message: 'Soup noodles are a good choice! They\'re light and easy to digest. Do you have any at home, or would you like me to help you order some?', 
            time: '2:20 PM',
            fullTime: '2025-11-25 14:20:33',
            timestamp: new Date('2025-11-25T14:20:33').getTime(),
            sentiment: 'neutral',
            emotionType: null
        },
        { 
            sender: 'elderly', 
            message: 'I think I have some instant noodles in the kitchen. That should be fine for now.', 
            time: '2:21 PM',
            fullTime: '2025-11-25 14:21:08',
            timestamp: new Date('2025-11-25T14:21:08').getTime(),
            sentiment: 'positive',
            emotionType: 'positive'
        },
        { 
            sender: 'robot', 
            message: 'Good! Just remember to be careful with hot water when preparing them. Would you like me to remind you about your medication schedule after you eat?', 
            time: '2:22 PM',
            fullTime: '2025-11-25 14:22:18',
            timestamp: new Date('2025-11-25T14:22:18').getTime(),
            sentiment: 'neutral',
            emotionType: null
        },
        { 
            sender: 'elderly', 
            message: 'Yes, that would be helpful. Thank you for reminding me!', 
            time: '2:23 PM',
            fullTime: '2025-11-25 14:23:02',
            timestamp: new Date('2025-11-25T14:23:02').getTime(),
            sentiment: 'positive',
            emotionType: 'positive'
        }
    ],
    conversationSessions: [
        {
            sessionId: 'session_001',
            startTime: '2025-11-25 14:15:32',
            endTime: '2025-11-25 14:23:02',
            duration: 450, // seconds (about 7.5 minutes)
            messageCount: 10,
            emotionDetected: ['positive', 'neutral'],
            concernsDetected: []
        }
    ],
    conversationSummary: {
        sessionsToday: 1,
        avgDuration: 7,
        keyTopics: ['Food', 'eating', 'noodles', 'medication reminder'],
        moodTrend: 'positive',
        concernsDetected: []
    },
    alertThresholds: {
        anxiety: 60,
        sadness: 50,
        happiness: 30,
        engagement: 40
    }
};

// Keywords for emotional analysis - Enhanced for emergency detection
const emotionalKeywords = {
    positive: ['happy', 'good', 'great', 'wonderful', 'excellent', 'tasty', 'nice', 'energetic', 'well', 'better', 'love', 'enjoy', 'fine', 'okay', 'alright'],
    negative: ['sad', 'lonely', 'tired', 'pain', 'hurt', 'miss', 'worry', 'scared', 'afraid', 'upset', 'angry', 'frustrated', 'bad', 'terrible', 'awful'],
    anxiety: ['worried', 'nervous', 'anxious', 'scared', 'afraid', 'concerned', 'stress', 'overwhelmed', 'panic', 'fear', 'worried about'],
    depression: ['lonely', 'alone', 'hopeless', 'worthless', 'empty', 'numb', 'don\'t care', 'give up', 'no point', 'meaningless'],
    emergency: ['help', 'emergency', 'urgent', 'immediate', 'now', 'can\'t breathe', 'chest pain', 'fall', 'fell', 'hurt myself', 'accident', '911', 'ambulance'],
    mentalHealth: ['depressed', 'suicidal', 'end it', 'kill myself', 'no reason to live', 'want to die', 'give up', 'hopeless', 'worthless']
};

// Analyze emotional state from chat message - Enhanced with emergency detection
function analyzeEmotionalState(message) {
    const messageLower = message.toLowerCase();
    let sentiment = {
        positive: 0,
        negative: 0,
        anxiety: 0,
        depression: 0,
        emergency: 0,
        mentalHealth: 0
    };

    // Count keyword matches
    for (const [emotion, keywords] of Object.entries(emotionalKeywords)) {
        keywords.forEach(keyword => {
            if (messageLower.includes(keyword)) {
                sentiment[emotion]++;
            }
        });
    }

    // Determine overall sentiment with priority: emergency > mental health > depression > anxiety > negative > positive
    if (sentiment.emergency >= 1 || sentiment.mentalHealth >= 2) {
        return { type: 'critical', score: sentiment, isEmergency: true };
    } else if (sentiment.mentalHealth >= 1) {
        return { type: 'critical', score: sentiment, isMentalHealth: true };
    } else if (sentiment.depression >= 2 || sentiment.negative >= 3) {
        return { type: 'critical', score: sentiment };
    } else if (sentiment.anxiety >= 2 || sentiment.negative >= 2) {
        return { type: 'warning', score: sentiment };
    } else if (sentiment.positive >= 2) {
        return { type: 'positive', score: sentiment };
    } else {
        return { type: 'neutral', score: sentiment };
    }
}

// Update emotional indicators based on conversation analysis
function updateEmotionalIndicators() {
    const recentMessages = mentalHealthData.chatHistory
        .filter(msg => msg.sender === 'elderly')
        .slice(-5); // Last 5 elderly messages

    let totalPositive = 0;
    let totalNegative = 0;
    let totalAnxiety = 0;

    recentMessages.forEach(msg => {
        const analysis = analyzeEmotionalState(msg.message);
        totalPositive += analysis.score.positive;
        totalNegative += analysis.score.negative;
        totalAnxiety += analysis.score.anxiety;
    });

    // Calculate percentages (0-100)
    mentalHealthData.currentState.happiness = Math.min(100, 50 + (totalPositive * 10) - (totalNegative * 5));
    mentalHealthData.currentState.anxiety = Math.min(100, 10 + (totalAnxiety * 10));
    mentalHealthData.currentState.sadness = Math.min(100, 5 + (totalNegative * 8));
    mentalHealthData.currentState.engagement = Math.min(100, 60 + (recentMessages.length * 5));

    // Update UI
    updateMentalHealthUI();

    // Check for alerts
    checkMentalHealthAlerts();
}

// Update Mental Health UI
function updateMentalHealthUI() {
    const state = mentalHealthData.currentState;

    // Update emotion bars
    const emotionBars = {
        happiness: document.querySelector('.emotion-fill.happiness'),
        anxiety: document.querySelector('.emotion-fill.anxiety'),
        sadness: document.querySelector('.emotion-fill.sadness'),
        engagement: document.querySelector('.emotion-fill.engagement')
    };

    if (emotionBars.happiness) emotionBars.happiness.style.width = state.happiness + '%';
    if (emotionBars.anxiety) emotionBars.anxiety.style.width = state.anxiety + '%';
    if (emotionBars.sadness) emotionBars.sadness.style.width = state.sadness + '%';
    if (emotionBars.engagement) emotionBars.engagement.style.width = state.engagement + '%';

    // Update values (formatted to 2 decimal places)
    const emotionValues = document.querySelectorAll('.emotion-value');
    if (emotionValues.length >= 4) {
        emotionValues[0].textContent = state.happiness.toFixed(2) + '%';
        emotionValues[1].textContent = state.anxiety.toFixed(2) + '%';
        emotionValues[2].textContent = state.sadness.toFixed(2) + '%';
        emotionValues[3].textContent = state.engagement.toFixed(2) + '%';
    }

    // Update overall assessment
    const assessmentBadge = document.querySelector('.assessment-badge');
    if (assessmentBadge) {
        assessmentBadge.className = 'assessment-badge';

        if (state.anxiety > mentalHealthData.alertThresholds.anxiety ||
            state.sadness > mentalHealthData.alertThresholds.sadness ||
            state.happiness < mentalHealthData.alertThresholds.happiness) {
            assessmentBadge.classList.add('alert');
            assessmentBadge.innerHTML = '<i class="fas fa-exclamation-triangle"></i><span>Mental Health Concerns Detected</span>';
        } else if (state.anxiety > 40 || state.sadness > 30) {
            assessmentBadge.classList.add('warning');
            assessmentBadge.innerHTML = '<i class="fas fa-exclamation-circle"></i><span>Monitor Mental State</span>';
        } else {
            assessmentBadge.classList.add('positive');
            assessmentBadge.innerHTML = '<i class="fas fa-check-circle"></i><span>Healthy Mental State</span>';
        }
    }

    // Update header status
    const mentalHealthStatus = document.getElementById('mentalHealthStatus');
    if (mentalHealthStatus) {
        if (state.happiness >= 70 && state.anxiety < 30 && state.sadness < 30) {
            mentalHealthStatus.innerHTML = '<i class="fas fa-smile"></i><span>Emotional State: <strong>Positive</strong></span>';
            mentalHealthStatus.style.background = 'rgba(107, 203, 119, 0.2)';
            mentalHealthStatus.style.color = '#2D7A3E';
        } else if (state.anxiety > 50 || state.sadness > 40) {
            mentalHealthStatus.innerHTML = '<i class="fas fa-frown"></i><span>Emotional State: <strong>Concerning</strong></span>';
            mentalHealthStatus.style.background = 'rgba(231, 111, 81, 0.2)';
            mentalHealthStatus.style.color = '#A84832';
        } else {
            mentalHealthStatus.innerHTML = '<i class="fas fa-meh"></i><span>Emotional State: <strong>Neutral</strong></span>';
            mentalHealthStatus.style.background = 'rgba(255, 179, 71, 0.2)';
            mentalHealthStatus.style.color = '#B87A00';
        }
    }
}

// Check for mental health alerts
function checkMentalHealthAlerts() {
    const state = mentalHealthData.currentState;
    const thresholds = mentalHealthData.alertThresholds;

    let alertType = null;
    let alertMessage = '';

    if (state.sadness > thresholds.sadness || state.happiness < thresholds.happiness) {
        alertType = 'critical';
        alertMessage = 'High levels of sadness detected in recent conversations. Immediate attention recommended.';
        mentalHealthData.conversationSummary.concernsDetected.push('Depression indicators');
    } else if (state.anxiety > thresholds.anxiety) {
        alertType = 'warning';
        alertMessage = 'Elevated anxiety levels detected. Consider scheduling a check-in with caregiver.';
        mentalHealthData.conversationSummary.concernsDetected.push('Anxiety indicators');
    } else if (state.engagement < thresholds.engagement) {
        alertType = 'warning';
        alertMessage = 'Low engagement in conversations. May indicate social withdrawal or fatigue.';
        mentalHealthData.conversationSummary.concernsDetected.push('Low engagement');
    }

    if (alertType) {
        showMentalHealthAlert(alertType, alertMessage);
    }
}

// Show mental health alert popup
function showMentalHealthAlert(type, message) {
    // Create overlay
    const overlay = document.createElement('div');
    overlay.className = 'alert-overlay';

    // Create alert box
    const alertBox = document.createElement('div');
    alertBox.className = `mental-health-alert ${type}`;

    const iconClass = type === 'critical' ? 'fa-exclamation-triangle' : 'fa-exclamation-circle';
    const iconColor = type === 'critical' ? 'critical' : 'warning';

    alertBox.innerHTML = `
        <div class="alert-header ${iconColor}">
            <i class="fas ${iconClass}"></i>
            <div>
                <h2>Mental Health Alert</h2>
                <p style="margin: 0; color: var(--text-secondary); font-size: 14px;">
                    ${type === 'critical' ? 'Critical' : 'Warning'} - Immediate Review Recommended
                </p>
            </div>
        </div>
        <div class="alert-body">
            <p style="color: var(--text-primary); line-height: 1.6; margin-bottom: 16px;">
                ${message}
            </p>
            <p style="color: var(--text-secondary); font-size: 14px; margin: 0;">
                <strong>Recommendation:</strong> Review recent chat history and consider contacting family member or healthcare provider.
            </p>
        </div>
        <div class="alert-actions">
            <button class="primary-action" onclick="viewChatHistory()">View Chat History</button>
            <button class="secondary-action" onclick="dismissMentalHealthAlert()">Dismiss</button>
        </div>
    `;

    document.body.appendChild(overlay);
    document.body.appendChild(alertBox);

    // Store references for dismissal
    window.currentAlertOverlay = overlay;
    window.currentAlertBox = alertBox;
}

// Dismiss mental health alert
function dismissMentalHealthAlert() {
    if (window.currentAlertOverlay) {
        window.currentAlertOverlay.remove();
    }
    if (window.currentAlertBox) {
        window.currentAlertBox.remove();
    }
}

// View chat history (scroll to chat section)
function viewChatHistory() {
    dismissMentalHealthAlert();
    const chatSection = document.querySelector('.chat-history-panel');
    if (chatSection) {
        chatSection.scrollIntoView({ behavior: 'smooth', block: 'center' });
        chatSection.style.boxShadow = '0 0 0 4px rgba(250, 114, 104, 0.3)';
        setTimeout(() => {
            chatSection.style.boxShadow = '';
        }, 2000);
    }
}

// Start new AI chat conversation with detailed tracking
function startAIChat() {
    showNotification('Starting AI conversation mode...', 'info');

    // Create new conversation session
    const sessionId = 'session_' + Date.now();
    const startTime = new Date();
    
    // Simulate adding new chat messages
    setTimeout(() => {
        const now = new Date();
        const newMessage = {
            sender: 'robot',
            message: 'Hello! I\'m here to chat. How are you feeling right now?',
            time: now.toLocaleTimeString('en-MY', { hour: '2-digit', minute: '2-digit' }),
            fullTime: formatFullTimestamp(now),
            timestamp: now.getTime(),
            sentiment: 'neutral',
            emotionType: null,
            sessionId: sessionId
        };

        mentalHealthData.chatHistory.push(newMessage);

        // Update chat display
        addChatMessage(newMessage);

        // Add to conversation sessions
        mentalHealthData.conversationSessions.push({
            sessionId: sessionId,
            startTime: formatFullTimestamp(startTime),
            endTime: null,
            duration: 0,
            messageCount: 1,
            emotionDetected: [],
            concernsDetected: []
        });

        showNotification('AI robot is ready to chat', 'success');
    }, 1000);
}

// Initialize conversation display with detailed timestamps
function initializeConversationDisplay() {
    const chatMessages = document.getElementById('chatMessages');
    if (!chatMessages) {
        console.log('chatMessages element not found');
        return;
    }
    
    console.log('Initializing conversation display with', mentalHealthData.chatHistory.length, 'messages');
    
    // Clear existing messages
    chatMessages.innerHTML = '';
    
    // Display all messages from chatHistory with full timestamps
    mentalHealthData.chatHistory.forEach((message, index) => {
        console.log('Adding message', index + 1, ':', message);
        addChatMessage(message);
    });
    
    // Update last chat time
    updateLastChatTime();
    
    console.log('Conversation display initialized');
}

// Add chat message to display with emotion detection
function addChatMessage(message) {
    const chatMessages = document.getElementById('chatMessages');
    if (!chatMessages) return;

    // Analyze emotion if it's from elderly
    let emotionAnalysis = null;
    let emotionClass = '';
    let emotionIndicator = '';
    
    if (message.sender === 'elderly') {
        emotionAnalysis = analyzeEmotionalState(message.message);
        message.emotionType = emotionAnalysis.type;
        message.sentiment = emotionAnalysis.type;
        
        if (emotionAnalysis.type === 'critical') {
            emotionClass = 'has-emotion-critical';
            if (emotionAnalysis.isEmergency) {
                emotionIndicator = '<span class="message-emotion-indicator critical"><i class="fas fa-exclamation-triangle"></i> EMERGENCY</span>';
            } else if (emotionAnalysis.isMentalHealth) {
                emotionIndicator = '<span class="message-emotion-indicator critical"><i class="fas fa-heartbeat"></i> MENTAL HEALTH</span>';
            } else {
                emotionIndicator = '<span class="message-emotion-indicator critical"><i class="fas fa-exclamation-circle"></i> CRITICAL</span>';
            }
        } else if (emotionAnalysis.type === 'warning') {
            emotionClass = 'has-emotion-warning';
            emotionIndicator = '<span class="message-emotion-indicator warning"><i class="fas fa-exclamation-circle"></i> WARNING</span>';
        } else if (emotionAnalysis.type === 'positive') {
            emotionClass = 'has-emotion-positive';
            emotionIndicator = '<span class="message-emotion-indicator positive"><i class="fas fa-smile"></i> POSITIVE</span>';
        }
        
        // Update mental health state
        updateEmotionalIndicators();
        
        // Check for emergency alerts
        if (emotionAnalysis.isEmergency || emotionAnalysis.isMentalHealth) {
            checkEmergencyAlerts(message, emotionAnalysis);
        }
    }

    const messageDiv = document.createElement('div');
    messageDiv.className = `chat-message ${message.sender}`;
    
    // Format full timestamp
    const fullTime = message.fullTime || formatFullTimestamp(new Date());

    if (message.sender === 'robot') {
        messageDiv.innerHTML = `
            <div class="message-avatar">
                <i class="fas fa-robot"></i>
            </div>
            <div class="message-content">
                <p>${message.message}</p>
                <span class="message-time">${message.time}</span>
                <span class="message-full-time">${fullTime}</span>
            </div>
        `;
    } else {
        messageDiv.innerHTML = `
            <div class="message-content ${emotionClass}">
                ${emotionIndicator}
                <p>${message.message}</p>
                <span class="message-time">${message.time}</span>
                <span class="message-full-time">${fullTime}</span>
            </div>
            <div class="message-avatar">
                <i class="fas fa-user"></i>
            </div>
        `;
    }

    chatMessages.appendChild(messageDiv);
    chatMessages.scrollTop = chatMessages.scrollHeight;
    
    // Update last chat time
    updateLastChatTime();
}

// Format full timestamp
function formatFullTimestamp(date) {
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    const hours = String(date.getHours()).padStart(2, '0');
    const minutes = String(date.getMinutes()).padStart(2, '0');
    const seconds = String(date.getSeconds()).padStart(2, '0');
    return `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`;
}

// Check for emergency alerts
function checkEmergencyAlerts(message, emotionAnalysis) {
    if (emotionAnalysis.isEmergency) {
        showNotification('🚨 EMERGENCY DETECTED in conversation! Immediate attention required.', 'warning');
        // Trigger emergency protocol
        triggerEmergencyFromChat(message);
    } else if (emotionAnalysis.isMentalHealth) {
        showNotification('⚠️ Mental health concern detected. Review conversation immediately.', 'warning');
        checkMentalHealthAlerts();
    }
}

// Trigger emergency from chat
function triggerEmergencyFromChat(message) {
    const alertStatus = document.getElementById('alertStatus');
    if (alertStatus) {
        alertStatus.innerHTML = '<i class="fas fa-exclamation-triangle"></i><span>EMERGENCY DETECTED</span>';
        alertStatus.style.background = 'rgba(231, 76, 60, 0.3)';
    }
    
    // Add to conversation summary concerns
    mentalHealthData.conversationSummary.concernsDetected.push({
        type: 'emergency',
        message: message.message,
        time: message.fullTime || formatFullTimestamp(new Date()),
        severity: 'critical'
    });
}

// Update last chat time
function updateLastChatTime() {
    const lastChatTime = document.querySelector('.last-chat-time');
    if (!lastChatTime) return;
    
    const now = new Date();
    const lastMessage = mentalHealthData.chatHistory[mentalHealthData.chatHistory.length - 1];
    
    if (lastMessage) {
        const lastTime = new Date(lastMessage.timestamp || now.getTime());
        const diffMs = now.getTime() - lastTime.getTime();
        const diffMins = Math.floor(diffMs / 60000);
        const diffHours = Math.floor(diffMs / 3600000);
        
        if (diffMins < 1) {
            lastChatTime.textContent = 'Last chat: Just now';
        } else if (diffMins < 60) {
            lastChatTime.textContent = `Last chat: ${diffMins} minute${diffMins > 1 ? 's' : ''} ago`;
        } else if (diffHours < 24) {
            lastChatTime.textContent = `Last chat: ${diffHours} hour${diffHours > 1 ? 's' : ''} ago`;
        } else {
            const diffDays = Math.floor(diffHours / 24);
            lastChatTime.textContent = `Last chat: ${diffDays} day${diffDays > 1 ? 's' : ''} ago`;
        }
    }
}

// Simulate mental health monitoring updates
function simulateMentalHealthUpdates() {
    // Randomly update emotional states slightly
    setInterval(() => {
        const state = mentalHealthData.currentState;

        // Small random variations
        state.happiness += (Math.random() - 0.5) * 3;
        state.anxiety += (Math.random() - 0.5) * 2;
        state.sadness += (Math.random() - 0.5) * 2;
        state.engagement += (Math.random() - 0.5) * 2;

        // Keep within bounds
        state.happiness = Math.max(0, Math.min(100, state.happiness));
        state.anxiety = Math.max(0, Math.min(100, state.anxiety));
        state.sadness = Math.max(0, Math.min(100, state.sadness));
        state.engagement = Math.max(0, Math.min(100, state.engagement));

        updateMentalHealthUI();
    }, 10000); // Update every 10 seconds
}

// Note: Conversation initialization is handled in main DOMContentLoaded event

// ===================================
// MOOD-ADAPTIVE HOME AUTOMATION
// ===================================

// Home automation settings
const homeAutomationSettings = {
    currentMood: 'positive', // positive, neutral, anxious, depressed
    autoAdjustments: {
        lighting: { brightness: 75, colorTemp: 'warm', active: true },
        temperature: { value: 24, fanSpeed: 'auto', active: true },
        music: { playing: true, type: 'classical', volume: 20 },
        doorLock: { locked: false, mode: 'standard' }
    },
    safetyProtocol: false
};

// Mood-based automation rules
const automationRules = {
    positive: {
        lighting: { brightness: 75, colorTemp: 'warm', reason: 'Positive mood - Comfortable ambiance' },
        temperature: { value: 24, fanSpeed: 'auto', reason: 'Relaxed state - Optimal comfort' },
        music: { playing: true, type: 'classical', volume: 20, reason: 'Calm mood - Relaxation enhancement' },
        doorLock: { locked: false, mode: 'standard', reason: 'Mental state healthy - Normal access' }
    },
    neutral: {
        lighting: { brightness: 65, colorTemp: 'neutral', reason: 'Neutral mood - Soft ambiance' },
        temperature: { value: 24, fanSpeed: 'auto', reason: 'Comfortable environment' },
        music: { playing: true, type: 'ambient', volume: 15, reason: 'Gentle background music' },
        doorLock: { locked: false, mode: 'standard', reason: 'Standard security mode' }
    },
    anxious: {
        lighting: { brightness: 50, colorTemp: 'cool', reason: 'Anxious state - Calming blue lighting' },
        temperature: { value: 22, fanSpeed: 'low', reason: 'Cooler temperature for stress relief' },
        music: { playing: true, type: 'meditation', volume: 25, reason: 'Meditation sounds activated' },
        doorLock: { locked: true, mode: 'safety', reason: 'Safety protocol - Doors locked to prevent wandering' }
    },
    depressed: {
        lighting: { brightness: 80, colorTemp: 'bright', reason: 'Therapeutic bright lighting for mood' },
        temperature: { value: 23, fanSpeed: 'auto', reason: 'Comfortable temperature' },
        music: { playing: true, type: 'uplifting', volume: 30, reason: 'Uplifting music for mood enhancement' },
        doorLock: { locked: true, mode: 'emergency', reason: 'Critical state - All exits secured for safety' }
    }
};

// Update mood-based automation
function updateMoodAdaptiveAutomation() {
    const state = mentalHealthData.currentState;
    let detectedMood = 'positive';

    // Determine mood based on emotional state
    if (state.sadness > 50 || state.happiness < 30) {
        detectedMood = 'depressed';
    } else if (state.anxiety > 60) {
        detectedMood = 'anxious';
    } else if (state.happiness >= 70 && state.anxiety < 30) {
        detectedMood = 'positive';
    } else {
        detectedMood = 'neutral';
    }

    // Update current mood
    homeAutomationSettings.currentMood = detectedMood;

    // Apply automation rules
    applyAutomationRules(detectedMood);

    // Update UI
    updateAutomationUI(detectedMood);

    // Check if safety protocol needs to be activated
    if (detectedMood === 'anxious' || detectedMood === 'depressed') {
        activateSafetyProtocol(detectedMood);
    } else {
        deactivateSafetyProtocol();
    }
}

// Apply automation rules based on mood
function applyAutomationRules(mood) {
    const rules = automationRules[mood];

    // Apply each rule
    homeAutomationSettings.autoAdjustments.lighting = {
        ...rules.lighting,
        active: true
    };

    homeAutomationSettings.autoAdjustments.temperature = {
        ...rules.temperature,
        active: true
    };

    homeAutomationSettings.autoAdjustments.music = {
        ...rules.music,
        active: true
    };

    homeAutomationSettings.autoAdjustments.doorLock = {
        ...rules.doorLock,
        active: true
    };

    // Log automation action
    console.log(`Mood-Adaptive Automation: Applied ${mood} settings`);

    // Show notification for significant changes
    if (mood === 'anxious' || mood === 'depressed') {
        showNotification(`Safety Protocol Activated - Environment adjusted for ${mood} state`, 'warning');
    }
}

// Update automation UI
function updateAutomationUI(mood) {
    // Update mood profile display
    const moodProfileDisplay = document.getElementById('moodProfileDisplay');
    if (moodProfileDisplay) {
        const moodIndicator = moodProfileDisplay.querySelector('.mood-indicator');
        if (moodIndicator) {
            moodIndicator.className = 'mood-indicator ' + getMoodClass(mood);

            const icons = {
                positive: 'fa-smile',
                neutral: 'fa-meh',
                anxious: 'fa-frown',
                depressed: 'fa-sad-tear'
            };

            const labels = {
                positive: 'Positive & Relaxed',
                neutral: 'Calm & Neutral',
                anxious: 'Anxious - Safety Active',
                depressed: 'Concerning - Enhanced Safety'
            };

            moodIndicator.innerHTML = `
                <i class="fas ${icons[mood]}"></i>
                <span>${labels[mood]}</span>
            `;
        }
    }

    // Update adjustments list
    updateAdjustmentsList();
}

// Update adjustments list display
function updateAdjustmentsList() {
    const adjustmentsList = document.getElementById('adjustmentsList');
    if (!adjustmentsList) return;

    const settings = homeAutomationSettings.autoAdjustments;

    // Update lighting
    const lightingItem = adjustmentsList.querySelector('.adjustment-item:nth-child(1)');
    if (lightingItem) {
        const info = lightingItem.querySelector('.adjustment-info');
        info.innerHTML = `
            <h4>Warm Lighting</h4>
            <p>Brightness: ${settings.lighting.brightness}% | Color Temp: ${settings.lighting.colorTemp}</p>
            <span class="adjustment-reason">Reason: ${settings.lighting.reason}</span>
        `;
    }

    // Update temperature
    const tempItem = adjustmentsList.querySelector('.adjustment-item:nth-child(2)');
    if (tempItem) {
        const info = tempItem.querySelector('.adjustment-info');
        info.innerHTML = `
            <h4>Climate Control</h4>
            <p>Temperature: ${settings.temperature.value}°C | Fan Speed: ${settings.temperature.fanSpeed}</p>
            <span class="adjustment-reason">Reason: ${settings.temperature.reason}</span>
        `;
    }

    // Update music
    const musicItem = adjustmentsList.querySelector('.adjustment-item:nth-child(3)');
    if (musicItem) {
        const info = musicItem.querySelector('.adjustment-info');
        const musicTypes = {
            classical: 'Soothing Classical',
            ambient: 'Ambient Sounds',
            meditation: 'Meditation Music',
            uplifting: 'Uplifting Melodies'
        };
        info.innerHTML = `
            <h4>Background Music</h4>
            <p>Playing: ${musicTypes[settings.music.type]} | Volume: ${settings.music.volume}%</p>
            <span class="adjustment-reason">Reason: ${settings.music.reason}</span>
        `;
    }

    // Update door lock
    const doorItem = adjustmentsList.querySelector('.adjustment-item:nth-child(4)');
    if (doorItem) {
        const info = doorItem.querySelector('.adjustment-info');
        const statusIcon = doorItem.querySelector('.adjustment-status');

        info.innerHTML = `
            <h4>Safety Door Lock</h4>
            <p>Status: ${settings.doorLock.locked ? 'Locked' : 'Unlocked'} | Mode: ${settings.doorLock.mode}</p>
            <span class="adjustment-reason">Reason: ${settings.doorLock.reason}</span>
        `;

        // Update status based on lock state
        doorItem.className = 'adjustment-item ' + (settings.doorLock.locked ? 'active' : 'standby');

        if (settings.doorLock.locked) {
            statusIcon.className = 'adjustment-status active';
            statusIcon.innerHTML = '<i class="fas fa-lock"></i>';
        } else {
            statusIcon.className = 'adjustment-status standby';
            statusIcon.innerHTML = '<i class="fas fa-unlock"></i>';
        }
    }
}

// Activate safety protocol
function activateSafetyProtocol(severity) {
    homeAutomationSettings.safetyProtocol = true;

    const banner = document.getElementById('safetyProtocolBanner');
    if (banner) {
        banner.style.display = 'flex';

        // Update banner content based on severity
        if (severity === 'depressed') {
            banner.querySelector('.protocol-content h3').textContent = 'Critical Safety Protocol Activated';
            banner.querySelector('.protocol-content p').textContent =
                'Severe mental health concerns detected. Enhanced safety measures enabled including full monitoring and emergency notifications.';
        } else {
            banner.querySelector('.protocol-content h3').textContent = 'Enhanced Safety Protocol Activated';
            banner.querySelector('.protocol-content p').textContent =
                'Mental health concerns detected. Automatic safety measures enabled to prevent wandering and ensure wellbeing.';
        }
    }

    // Update IoT device states on main dashboard
    updateIoTDeviceStates();
}

// Deactivate safety protocol
function deactivateSafetyProtocol() {
    homeAutomationSettings.safetyProtocol = false;

    const banner = document.getElementById('safetyProtocolBanner');
    if (banner) {
        banner.style.display = 'none';
    }

    updateIoTDeviceStates();
}

// Update IoT device states based on automation
function updateIoTDeviceStates() {
    // This would update the actual IoT devices in the Smart Home section
    // For demo purposes, we'll update the UI elements

    const doorDevice = document.querySelector('.iot-device .device-info h4:contains("Smart Door Lock")');
    if (doorDevice && homeAutomationSettings.autoAdjustments.doorLock.locked) {
        const deviceInfo = doorDevice.nextElementSibling;
        if (deviceInfo) {
            deviceInfo.textContent = 'Locked - Safety Mode';
        }
    }
}

// Get mood class for styling
function getMoodClass(mood) {
    const moodClasses = {
        positive: 'positive',
        neutral: 'neutral',
        anxious: 'warning',
        depressed: 'warning'
    };
    return moodClasses[mood] || 'neutral';
}

// Override checkMentalHealthAlerts to integrate with automation
const originalCheckMentalHealthAlerts = checkMentalHealthAlerts;
checkMentalHealthAlerts = function() {
    // Call original function
    originalCheckMentalHealthAlerts();

    // Update mood-adaptive automation
    updateMoodAdaptiveAutomation();
};

// Initialize mood-adaptive automation if elements exist
if (document.getElementById('moodProfileDisplay')) {
    updateMoodAdaptiveAutomation();

    // Update automation every 15 seconds
    setInterval(updateMoodAdaptiveAutomation, 15000);
}

// ===================================
// EXPORT FUNCTIONS (for other pages)
// ===================================
window.dashboardUtils = {
    elderlyData,
    generateTimeLabels,
    generateHeartRateData,
    generateBloodPressureData,
    generateBloodSugarData,
    generateTemperatureData,
    showNotification,
    navigateTo
};
