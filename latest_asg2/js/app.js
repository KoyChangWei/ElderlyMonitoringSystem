// ================================
// ELDERCARE AI MONITORING SYSTEM
// Main JavaScript - Data & Logic
// ================================

// === INITIALIZE LOCALSTORAGE ===
function initializeLocalStorage() {
    if (!localStorage.getItem('elderlyResidents')) {
        localStorage.setItem('elderlyResidents', JSON.stringify(defaultElderlyList));
    }
}

// === LOAD RESIDENTS FROM LOCALSTORAGE ===
function loadElderlyList() {
    const stored = localStorage.getItem('elderlyResidents');
    return stored ? JSON.parse(stored) : defaultElderlyList;
}

// === DEFAULT ELDERLY RESIDENTS DATA ===
const defaultElderlyList = [
    {
        id: 1,
        personal: {
            name: 'Ah Chong',
            fullName: 'Ah Chong Tan',
            age: 78,
            room: 'Room A',
            color: '#3B82F6',
            wristbandId: 'WB-001',
            conditions: ['Hypertension', 'Diabetes']
        },
        vitals: {
            heartRate: { current: 72, normal: '60-100', unit: 'bpm', status: 'normal' },
            bloodPressure: { systolic: 145, diastolic: 90, unit: 'mmHg', status: 'warning' },
            bloodSugar: { current: 6.2, normal: '4.0-7.0', unit: 'mmol/L', status: 'normal' },
            temperature: { current: 36.8, normal: '36.5-37.5', unit: '°C', status: 'normal' }
        },
        activity: {
            steps: 3245,
            stepGoal: 5000,
            sleepHours: 7.2,
            sleepQuality: 'Good',
            lastFall: 'None',
            location: 'Living Room',
            aiPrediction: 85
        },
        alerts: [
            { type: 'warning', message: 'High Blood Pressure detected', time: '2 min ago' }
        ]
    },
    {
        id: 2,
        personal: {
            name: 'Mei Ling',
            fullName: 'Mei Ling Wong',
            age: 72,
            room: 'Room B',
            color: '#8B5CF6',
            wristbandId: 'WB-002',
            conditions: ['Arthritis']
        },
        vitals: {
            heartRate: { current: 68, normal: '60-100', unit: 'bpm', status: 'normal' },
            bloodPressure: { systolic: 128, diastolic: 82, unit: 'mmHg', status: 'normal' },
            bloodSugar: { current: 5.8, normal: '4.0-7.0', unit: 'mmol/L', status: 'normal' },
            temperature: { current: 37.1, normal: '36.5-37.5', unit: '°C', status: 'normal' }
        },
        activity: {
            steps: 1823,
            stepGoal: 5000,
            sleepHours: 6.5,
            sleepQuality: 'Fair',
            lastFall: 'None',
            location: 'Bedroom',
            aiPrediction: 92
        },
        alerts: [
            { type: 'info', message: 'Low activity level today', time: '15 min ago' }
        ]
    },
    {
        id: 3,
        personal: {
            name: 'Kumar',
            fullName: 'Kumar Raj',
            age: 81,
            room: 'Room C',
            color: '#14B8A6',
            wristbandId: 'WB-003',
            conditions: ['Heart Disease', 'Dementia']
        },
        vitals: {
            heartRate: { current: 76, normal: '60-100', unit: 'bpm', status: 'normal' },
            bloodPressure: { systolic: 132, diastolic: 85, unit: 'mmHg', status: 'normal' },
            bloodSugar: { current: 5.4, normal: '4.0-7.0', unit: 'mmol/L', status: 'normal' },
            temperature: { current: 36.6, normal: '36.5-37.5', unit: '°C', status: 'normal' }
        },
        activity: {
            steps: 4521,
            stepGoal: 5000,
            sleepHours: 8.1,
            sleepQuality: 'Excellent',
            lastFall: 'None',
            location: 'Garden',
            aiPrediction: 78
        },
        alerts: []
    },
    {
        id: 4,
        personal: {
            name: 'Fatimah',
            fullName: 'Fatimah Ibrahim',
            age: 75,
            room: 'Room D',
            color: '#F59E0B',
            wristbandId: 'WB-004',
            conditions: ['Osteoporosis']
        },
        vitals: {
            heartRate: { current: 70, normal: '60-100', unit: 'bpm', status: 'normal' },
            bloodPressure: { systolic: 135, diastolic: 88, unit: 'mmHg', status: 'normal' },
            bloodSugar: { current: 6.0, normal: '4.0-7.0', unit: 'mmol/L', status: 'normal' },
            temperature: { current: 37.0, normal: '36.5-37.5', unit: '°C', status: 'normal' }
        },
        activity: {
            steps: 5243,
            stepGoal: 5000,
            sleepHours: 7.8,
            sleepQuality: 'Good',
            lastFall: 'None',
            location: 'Dining Room',
            aiPrediction: 95
        },
        alerts: [
            { type: 'info', message: 'Medication due in 30 minutes', time: 'Just now' }
        ]
    }
];

// === ROBOT SCHEDULE DATA ===
const robotSchedule = [
    { time: '08:00', task: '🍳 Cook Meal', resident: 'Ah Chong', residentColor: '#3B82F6', status: 'done', repeat: 'Daily' },
    { time: '09:30', task: '💊 Medication Reminder', resident: 'Mei Ling', residentColor: '#8B5CF6', status: 'done', repeat: 'Daily' },
    { time: '10:45', task: '💧 Bring Water', resident: 'Kumar', residentColor: '#14B8A6', status: 'active', repeat: 'As needed' },
    { time: '12:00', task: '🍳 Cook Meal', resident: 'Fatimah', residentColor: '#F59E0B', status: 'pending', repeat: 'Daily' },
    { time: '14:30', task: '❤️ Check Vitals', resident: 'Ah Chong', residentColor: '#3B82F6', status: 'pending', repeat: 'Daily' },
    { time: '16:00', task: '🚶 Assist Walking', resident: 'Kumar', residentColor: '#14B8A6', status: 'pending', repeat: 'Mon, Wed, Fri' }
];

// === MEDICATION SCHEDULE DATA ===
const medicationSchedule = [
    { time: '08:00', medication: 'Lisinopril 10mg', resident: 'Ah Chong', residentColor: '#3B82F6', status: 'completed', countdown: null },
    { time: '09:00', medication: 'Metformin 500mg', resident: 'Ah Chong', residentColor: '#3B82F6', status: 'completed', countdown: null },
    { time: '09:30', medication: 'Ibuprofen 400mg', resident: 'Mei Ling', residentColor: '#8B5CF6', status: 'completed', countdown: null },
    { time: '11:15', medication: 'Aspirin 75mg', resident: 'Kumar', residentColor: '#14B8A6', status: 'upcoming', countdown: '30 min' },
    { time: '12:30', medication: 'Calcium 600mg', resident: 'Fatimah', residentColor: '#F59E0B', status: 'upcoming', countdown: '1h 45min' },
    { time: '14:00', medication: 'Vitamin D 1000IU', resident: 'Mei Ling', residentColor: '#8B5CF6', status: 'pending', countdown: null },
    { time: '18:00', medication: 'Lisinopril 10mg', resident: 'Ah Chong', residentColor: '#3B82F6', status: 'pending', countdown: null },
    { time: '20:00', medication: 'Donepezil 5mg', resident: 'Kumar', residentColor: '#14B8A6', status: 'pending', countdown: null }
];

// === MEDICATION & HEALTH EVENTS (for calendar) ===
const medicationEvents = [
    {
        date: '2025-12-05',
        title: 'Cardiology Appointment',
        time: '10:00 AM',
        type: 'Cardiology',
        resident: 'Kumar Raj'
    },
    {
        date: '2025-12-12',
        title: 'Annual Health Checkup',
        time: '2:00 PM',
        type: 'Health Checkup',
        resident: 'Mei Ling Wong'
    },
    {
        date: '2025-12-20',
        title: 'Blood Test',
        time: '9:00 AM',
        type: 'Lab Test',
        resident: 'Ah Chong Tan'
    }
];

// === ACTIVITY FEED DATA ===
let activityFeed = [
    {
        icon: '💊',
        iconBg: '#8B5CF6',
        title: 'Medication Taken',
        description: 'Mei Ling Wong took Ibuprofen 400mg',
        time: '5 min ago',
        borderColor: '#8B5CF6',
        resident: 'Mei Ling Wong'
    },
    {
        icon: '🍽️',
        iconBg: '#3B82F6',
        title: 'Meal Logged',
        description: 'Ah Chong Tan finished breakfast',
        time: '12 min ago',
        borderColor: '#3B82F6',
        resident: 'Ah Chong Tan'
    },
    {
        icon: '🚶',
        iconBg: '#14B8A6',
        title: 'Activity Alert',
        description: 'Kumar Raj started morning walk',
        time: '25 min ago',
        borderColor: '#14B8A6',
        resident: 'Kumar Raj'
    },
    {
        icon: '❤️',
        iconBg: '#F59E0B',
        title: 'Vitals Check',
        description: 'Fatimah Ibrahim - All vitals normal',
        time: '42 min ago',
        borderColor: '#F59E0B',
        resident: 'Fatimah Ibrahim'
    },
    {
        icon: '💧',
        iconBg: '#14B8A6',
        title: 'Hydration',
        description: 'Kumar Raj drank water (250ml)',
        time: '1 hour ago',
        borderColor: '#14B8A6',
        resident: 'Kumar Raj'
    }
];

// === CHART DATA (24 Hours) ===
let healthChartData = {
    labels: ['00:00', '03:00', '06:00', '09:00', '12:00', '15:00', '18:00', '21:00', '24:00'],
    heartRate: {
        'Ah Chong': [68, 70, 72, 75, 73, 72, 74, 71, 72],
        'Mei Ling': [65, 66, 68, 70, 69, 68, 67, 66, 68],
        'Kumar': [72, 74, 76, 78, 77, 76, 75, 74, 76],
        'Fatimah': [67, 68, 70, 72, 71, 70, 69, 68, 70]
    },
    bloodPressure: {
        'Ah Chong': [142, 145, 148, 150, 147, 145, 144, 143, 145],
        'Mei Ling': [125, 126, 128, 130, 129, 128, 127, 126, 128],
        'Kumar': [130, 131, 132, 134, 133, 132, 131, 130, 132],
        'Fatimah': [132, 133, 135, 137, 136, 135, 134, 133, 135]
    },
    bloodSugar: {
        'Ah Chong': [5.8, 6.0, 6.2, 6.5, 6.3, 6.2, 6.1, 6.0, 6.2],
        'Mei Ling': [5.5, 5.6, 5.8, 6.0, 5.9, 5.8, 5.7, 5.6, 5.8],
        'Kumar': [5.2, 5.3, 5.4, 5.6, 5.5, 5.4, 5.3, 5.2, 5.4],
        'Fatimah': [5.8, 5.9, 6.0, 6.2, 6.1, 6.0, 5.9, 5.8, 6.0]
    },
    temperature: {
        'Ah Chong': [36.6, 36.7, 36.8, 37.0, 36.9, 36.8, 36.7, 36.6, 36.8],
        'Mei Ling': [36.9, 37.0, 37.1, 37.3, 37.2, 37.1, 37.0, 36.9, 37.1],
        'Kumar': [36.5, 36.5, 36.6, 36.8, 36.7, 36.6, 36.5, 36.5, 36.6],
        'Fatimah': [36.8, 36.9, 37.0, 37.2, 37.1, 37.0, 36.9, 36.8, 37.0]
    }
};

// === GLOBAL VARIABLES ===
let healthChart = null;
let activityFeedFilter = 'all';

// === GLOBAL ELDERLY LIST ===
let elderlyList = [];

// === INITIALIZE LOCALSTORAGE ON LOAD ===
initializeLocalStorage();
elderlyList = loadElderlyList();

// === EXPOSE DATA FOR OTHER PAGES ===
window.elderlyData = {
    elderlyList: elderlyList,
    healthChartData: healthChartData,
    robotSchedule: robotSchedule,
    medicationSchedule: medicationSchedule,
    medicationEvents: medicationEvents,
    activityFeed: activityFeed
};

// === INITIALIZATION ===
document.addEventListener('DOMContentLoaded', function() {
    console.log('ElderCare AI Dashboard Initializing...');

    // Initialize all components
    initializeStickyHeader();
    renderResidentsList();
    renderAlertsBanner();
    renderHealthTable();
    renderRobotSchedule();
    renderActivityTable();
    renderMedicationList();
    renderActivityFeed();
    initializeChart();
    updateCurrentTime();

    // Start real-time updates
    setInterval(updateCurrentTime, 1000);
    setInterval(simulateRealTimeUpdates, 5000);
    setInterval(addActivityFeedItem, 30000);

    console.log('ElderCare AI Dashboard Ready!');
});

// === HEADER & SIDEBAR TRANSITIONS ===
function initializeStickyHeader() {
    const header = document.querySelector('.sticky-top-header');
    const sidebar = document.querySelector('.sidebar');
    if (!header) return;

    // Ensure CSS variable matches actual header height
    const headerHeight = header.offsetHeight || 80;
    document.documentElement.style.setProperty('--header-height', `${headerHeight}px`);

    let lastScrollTop = 0;
    let ticking = false;
    const showThreshold = 50; // pixels of upward scroll needed to show header
    const hideThreshold = 100; // pixels scrolled down before hiding header

    const updateHeaderAndSidebar = () => {
        const currentScroll = window.pageYOffset || document.documentElement.scrollTop;
        const scrollDiff = currentScroll - lastScrollTop;

        // Only proceed if we've scrolled past the hide threshold
        if (currentScroll > hideThreshold) {
            if (scrollDiff > 5) {
                // Scrolling down - hide header
                if (!header.classList.contains('hidden')) {
                    header.classList.add('hidden');
                    document.body.classList.add('header-hidden');
                }
            } else if (scrollDiff < -showThreshold) {
                // Scrolling up significantly - show header
                if (header.classList.contains('hidden')) {
                    header.classList.remove('hidden');
                    document.body.classList.remove('header-hidden');
                }
            }
        } else {
            // At top of page - always show header
            if (header.classList.contains('hidden')) {
                header.classList.remove('hidden');
                document.body.classList.remove('header-hidden');
            }
        }

        lastScrollTop = currentScroll;
        ticking = false;
    };

    window.addEventListener(
        'scroll',
        () => {
            if (!ticking) {
                window.requestAnimationFrame(updateHeaderAndSidebar);
                ticking = true;
            }
        },
        { passive: true }
    );
}

// === RENDER FUNCTIONS ===

function renderResidentsList() {
    const container = document.getElementById('residentsList');
    if (!container) return;

    container.innerHTML = elderlyList.map(resident => `
        <div class="resident-card" style="border-left-color: ${resident.personal.color}" onclick="selectResident(${resident.id})">
            <div class="resident-header">
                <div class="resident-name">
                    <span class="status-indicator ${resident.vitals.heartRate.status}"></span>
                    ${resident.personal.fullName}
                </div>
            </div>
            <div class="resident-info">
                <span>Age: ${resident.personal.age}</span>
                <span>HR: ${resident.vitals.heartRate.current} ${resident.vitals.heartRate.unit}</span>
            </div>
        </div>
    `).join('');
}

function renderAlertsBanner() {
    const container = document.getElementById('alertsBanner');
    if (!container) return;

    const allAlerts = [];
    elderlyList.forEach(resident => {
        resident.alerts.forEach(alert => {
            allAlerts.push({
                ...alert,
                resident: resident.personal.fullName,
                color: resident.personal.color
            });
        });
    });

    if (allAlerts.length === 0) {
        container.style.display = 'none';
        return;
    }

    container.innerHTML = allAlerts.map((alert, index) => `
        <div class="alert alert-${alert.type}" style="border-left-color: ${alert.color}">
            <div class="alert-content">
                <div class="alert-icon">
                    ${alert.type === 'critical' ? '🚨' : alert.type === 'warning' ? '⚠️' : 'ℹ️'}
                </div>
                <div class="alert-text">
                    <strong>${alert.resident}: ${alert.message}</strong>
                    <span>${alert.time}</span>
                </div>
            </div>
            <button class="btn-dismiss" onclick="dismissAlert(${index})">Dismiss</button>
        </div>
    `).join('');
}

function renderHealthTable() {
    const tbody = document.getElementById('healthTable');
    if (!tbody) return;

    tbody.innerHTML = elderlyList.map(resident => `
        <tr>
            <td>
                <div class="resident-name-cell">
                    <div class="resident-color-dot" style="background: ${resident.personal.color}"></div>
                    ${resident.personal.name}
                </div>
            </td>
            <td>
                <span class="status-badge status-${resident.vitals.heartRate.status}">
                    ${resident.vitals.heartRate.current} bpm
                </span>
            </td>
            <td>
                <span class="status-badge status-${resident.vitals.bloodPressure.status}">
                    ${resident.vitals.bloodPressure.systolic}/${resident.vitals.bloodPressure.diastolic}
                </span>
            </td>
            <td>${resident.vitals.bloodSugar.current} ${resident.vitals.bloodSugar.unit}</td>
            <td>${resident.vitals.temperature.current} ${resident.vitals.temperature.unit}</td>
            <td>
                <div style="display: flex; align-items: center; gap: 8px;">
                    <div class="progress-bar" style="width: 100px;">
                        <div class="progress-fill" style="width: ${resident.activity.aiPrediction}%; background: ${resident.activity.aiPrediction > 85 ? '#10b981' : resident.activity.aiPrediction > 70 ? '#f59e0b' : '#ef4444'}"></div>
                    </div>
                    <span style="font-weight: 600;">${resident.activity.aiPrediction}%</span>
                </div>
            </td>
        </tr>
    `).join('');
}

function renderRobotSchedule() {
    const tbody = document.getElementById('robotScheduleTable');
    if (!tbody) return;

    const upcomingTasks = robotSchedule.slice(0, 4);

    tbody.innerHTML = upcomingTasks.map(task => `
        <tr>
            <td style="font-weight: 600;">${task.time}</td>
            <td>${task.task}</td>
            <td>
                <div class="resident-name-cell">
                    <div class="resident-color-dot" style="background: ${task.residentColor}"></div>
                    ${task.resident}
                </div>
            </td>
            <td>
                <span class="status-badge status-${task.status}">
                    ${task.status === 'done' ? '✓ Done' : task.status === 'active' ? '● Active' : 'Pending'}
                </span>
            </td>
        </tr>
    `).join('');
}

function renderActivityTable() {
    const tbody = document.getElementById('activityTable');
    if (!tbody) return;

    tbody.innerHTML = elderlyList.map(resident => `
        <tr>
            <td>
                <div class="resident-name-cell">
                    <div class="resident-color-dot" style="background: ${resident.personal.color}"></div>
                    ${resident.personal.name}
                </div>
            </td>
            <td>${resident.activity.steps.toLocaleString()}</td>
            <td>${resident.activity.sleepHours}h (${resident.activity.sleepQuality})</td>
            <td><span class="status-badge status-normal">0 Falls</span></td>
            <td>${resident.activity.location}</td>
        </tr>
    `).join('');
}

function renderMedicationList() {
    const container = document.getElementById('medicationList');
    if (!container) return;

    // Show all upcoming medications (not just 5) so user can scroll through them
    const upcomingMeds = medicationSchedule.filter(med => med.status !== 'completed');

    container.innerHTML = upcomingMeds.map(med => `
        <div class="medication-item" style="border-left-color: ${med.residentColor}">
            <div class="medication-header">
                <span class="medication-time">${med.time}</span>
                <span class="status-badge status-${med.status === 'completed' ? 'done' : med.status === 'upcoming' ? 'active' : 'pending'}">
                    ${med.status}
                </span>
            </div>
            <div class="medication-info">
                <strong>${med.medication}</strong> - ${med.resident}
            </div>
            ${med.countdown ? `<div class="medication-countdown">Due in ${med.countdown}</div>` : ''}
        </div>
    `).join('');
}

function renderActivityFeed() {
    const container = document.getElementById('activityFeed');
    if (!container) return;
    container.innerHTML = activityFeed.map(activity => `
        <div class="activity-item" style="border-left-color: ${activity.borderColor}">
            <div class="activity-icon" style="background: ${activity.iconBg}">
                ${activity.icon}
            </div>
            <div class="activity-details">
                <div class="activity-title">${activity.title}</div>
                <div class="activity-description">${activity.description}</div>
                <div class="activity-time">${activity.time}</div>
            </div>
        </div>
    `).join('') || `<div style="font-size: 12px; color: var(--text-secondary);">No recent activities.</div>`;
}

// === CHART FUNCTIONS ===

function initializeChart() {
    const ctx = document.getElementById('healthTrendChart');
    if (!ctx) return;

    const metric = document.getElementById('chartMetric')?.value || 'heartRate';

    const datasets = elderlyList.map(resident => ({
        label: resident.personal.fullName,
        data: healthChartData[metric][resident.personal.name],
        borderColor: resident.personal.color,
        backgroundColor: resident.personal.color + '20',
        borderWidth: 2,
        tension: 0.4,
        fill: true
    }));

    healthChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: healthChartData.labels,
            datasets: datasets
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: true,
                    position: 'bottom',
                    labels: {
                        color: '#0f172a',
                        usePointStyle: true,
                        padding: 15,
                        font: {
                            size: 12,
                            weight: '600'
                        }
                    }
                }
            },
            scales: {
                y: {
                    beginAtZero: false,
                    grid: {
                        color: 'rgba(255, 255, 255, 0.1)'
                    },
                    ticks: {
                        color: '#94a3b8'
                    }
                },
                x: {
                    grid: {
                        color: 'rgba(255, 255, 255, 0.1)'
                    },
                    ticks: {
                        color: '#94a3b8'
                    }
                }
            }
        }
    });
}

function updateChart() {
    if (!healthChart) return;

    const metric = document.getElementById('chartMetric').value;

    const datasets = elderlyList.map(resident => ({
        label: resident.personal.fullName,
        data: healthChartData[metric][resident.personal.name],
        borderColor: resident.personal.color,
        backgroundColor: resident.personal.color + '20',
        borderWidth: 2,
        tension: 0.4,
        fill: true
    }));

    healthChart.data.datasets = datasets;
    healthChart.update();
}

// === UTILITY FUNCTIONS ===

function updateCurrentTime() {
    const timeElement = document.getElementById('currentTime');
    if (timeElement) {
        const now = new Date();
        timeElement.textContent = now.toLocaleString('en-US', {
            month: 'short',
            day: 'numeric',
            year: 'numeric',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit'
        });
    }
}

function simulateRealTimeUpdates() {
    // Randomly update heart rates within normal range
    elderlyList.forEach(resident => {
        const variation = Math.floor(Math.random() * 5) - 2;
        resident.vitals.heartRate.current = Math.max(60, Math.min(100, resident.vitals.heartRate.current + variation));
    });

    // Update displays
    renderResidentsList();
    renderHealthTable();
}

function addActivityFeedItem() {
    const newActivities = [
        { icon: '💊', iconBg: '#8B5CF6', title: 'Medication Taken', description: 'Medication dose completed', time: 'Just now', borderColor: '#8B5CF6' },
        { icon: '🚶', iconBg: '#14B8A6', title: 'Activity Detected', description: 'Walking activity recorded', time: 'Just now', borderColor: '#14B8A6' },
        { icon: '❤️', iconBg: '#F59E0B', title: 'Vitals Check', description: 'Regular vitals monitoring', time: 'Just now', borderColor: '#F59E0B' }
    ];

    const randomActivity = newActivities[Math.floor(Math.random() * newActivities.length)];
    activityFeed.unshift(randomActivity);

    if (activityFeed.length > 10) {
        activityFeed = activityFeed.slice(0, 10);
    }

    renderActivityFeed();
}

// === EVENT HANDLERS ===

function navigateTo(page) {
    window.location.href = page;
}

function selectResident(id) {
    console.log('Selected resident:', id);
    // Could implement filtering or highlighting
}

function dismissAlert(index) {
    const allAlerts = [];
    elderlyList.forEach(resident => {
        resident.alerts.forEach(alert => {
            allAlerts.push({ alert, resident });
        });
    });

    if (allAlerts[index]) {
        const alertData = allAlerts[index];
        const residentIndex = elderlyList.findIndex(r => r === alertData.resident);
        const alertIndex = elderlyList[residentIndex].alerts.findIndex(a => a === alertData.alert);
        elderlyList[residentIndex].alerts.splice(alertIndex, 1);
        renderAlertsBanner();
    }
}

function toggleDevice(deviceId) {
    const checkbox = document.getElementById(`device-${deviceId}`);
    console.log(`${deviceId} is now ${checkbox.checked ? 'ON' : 'OFF'}`);
    // Prevent navigation when clicking toggle
    event.stopPropagation();
}

function callContact(contactType) {
    alert(`Calling ${contactType}...`);
}

function triggerEmergency() {
    if (confirm('Are you sure you want to trigger an emergency alert?')) {
        alert('🚨 EMERGENCY ALERT ACTIVATED!\n\n- Calling Emergency Services (999)\n- Alerting all family members\n- Activating all cameras\n- Unlocking all doors\n- Sending location to responders');
    }
}

// === EXPORT FOR OTHER PAGES ===
if (typeof window !== 'undefined') {
    window.elderlyData = {
        elderlyList,
        robotSchedule,
        medicationSchedule,
        medicationEvents,
        activityFeed,
        healthChartData
    };
}
