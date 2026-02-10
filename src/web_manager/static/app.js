/**
 * Husky Manager - Frontend JavaScript
 * Handles real-time updates and user interactions
 */

// Configuration
const CONFIG = {
    updateInterval: 1000,  // Update every 1 second
    apiBase: '/manager/api'
};

// State
let isConnected = false;
let updateTimer = null;

// ============================================
// Utility Functions
// ============================================

function showToast(message, type = 'info') {
    const container = document.getElementById('toast-container');
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.innerHTML = `
        <span class="toast-icon">${type === 'success' ? '✓' : type === 'error' ? '✗' : 'ℹ'}</span>
        <span class="toast-message">${message}</span>
    `;
    container.appendChild(toast);
    
    setTimeout(() => {
        toast.style.animation = 'slideIn 0.3s ease reverse';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

function updateDateTime() {
    const now = new Date();
    const options = { 
        hour: '2-digit', 
        minute: '2-digit', 
        day: '2-digit', 
        month: '2-digit', 
        year: 'numeric' 
    };
    document.getElementById('datetime').textContent = now.toLocaleDateString('es-ES', options);
}

function setConnectionStatus(connected) {
    isConnected = connected;
    const indicator = document.getElementById('connection-status');
    indicator.className = `status-indicator ${connected ? 'connected' : 'disconnected'}`;
    indicator.querySelector('.text').textContent = connected ? 'Connected' : 'Disconnected';
}

// ============================================
// API Functions
// ============================================

async function fetchWithTimeout(url, options = {}, timeout = 5000) {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), timeout);
    
    try {
        const response = await fetch(url, {
            ...options,
            signal: controller.signal,
            cache: 'no-store'  // Force no caching
        });
        clearTimeout(timeoutId);
        return response;
    } catch (error) {
        clearTimeout(timeoutId);
        // Improve error message based on error type
        if (error.name === 'AbortError') {
            throw new Error(`Request timeout after ${timeout/1000}s`);
        }
        throw error;
    }
}

async function fetchStatus() {
    try {
        const response = await fetchWithTimeout(`${CONFIG.apiBase}/status`, {}, 8000);  // 8s timeout for status
        if (!response.ok) throw new Error('Network response was not ok');
        
        const data = await response.json();
        setConnectionStatus(true);
        return data;
    } catch (error) {
        // Only log if not a repeated disconnect
        if (isConnected) {
            console.error('Failed to fetch status:', error.message || error);
        }
        setConnectionStatus(false);
        return null;
    }
}

async function relaunchProcess(name) {
    try {
        const response = await fetchWithTimeout(`${CONFIG.apiBase}/processes/${encodeURIComponent(name)}/relaunch`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({})
        }, 30000);  // 30s timeout for restart
        
        const result = await response.json();
        
        if (result.success) {
            showToast(`${name} restarted successfully`, 'success');
        } else {
            showToast(`Failed to restart ${name}: ${result.message}`, 'error');
        }
        
        return result;
    } catch (error) {
        console.error('Failed to relaunch process:', error);
        showToast(`Error restarting ${name}`, 'error');
        return { success: false };
    }
}

async function stopProcess(name) {
    try {
        const response = await fetchWithTimeout(`${CONFIG.apiBase}/processes/${encodeURIComponent(name)}/stop`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({})
        }, 90000);  // 90s timeout for graceful stop
        
        const result = await response.json();
        
        if (result.success) {
            showToast(`${name} stopped successfully`, 'success');
        } else {
            showToast(`Failed to stop ${name}: ${result.message}`, 'error');
        }
        
        return result;
    } catch (error) {
        console.error('Failed to stop process:', error);
        showToast(`Error stopping ${name}`, 'error');
        return { success: false };
    }
}

async function fetchLogs(name, lines = 150) {
    try {
        const response = await fetchWithTimeout(`${CONFIG.apiBase}/processes/${encodeURIComponent(name)}/logs?lines=${lines}`);
        if (!response.ok) throw new Error('Network response was not ok');
        
        const data = await response.json();
        return data;
    } catch (error) {
        console.error('Failed to fetch logs:', error);
        return { success: false, message: error.message, logs: '' };
    }
}

async function toggleTopic(group, name) {
    try {
        const response = await fetchWithTimeout(`${CONFIG.apiBase}/topics/${encodeURIComponent(group)}/${encodeURIComponent(name)}/toggle`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({})
        });
        
        const result = await response.json();
        
        if (result.success) {
            showToast(`${name}: ${result.value ? 'ON' : 'OFF'}`, 'success');
        } else {
            showToast(`Failed to toggle ${name}: ${result.message}`, 'error');
        }
        
        return result;
    } catch (error) {
        console.error('Failed to toggle topic:', error);
        showToast(`Error toggling ${name}`, 'error');
        return { success: false };
    }
}

async function handleToggle(group, name) {
    await toggleTopic(group, name);
    // Force immediate update
    await updateDashboard();
}

// ============================================
// Log Modal Functions
// ============================================

let currentLogProcess = null;

function showLogModal(name) {
    currentLogProcess = name;
    const modal = document.getElementById('log-modal');
    const title = document.getElementById('log-modal-title');
    const content = document.getElementById('log-modal-content');
    
    title.textContent = `📋 Logs: ${name}`;
    content.textContent = 'Loading logs...';
    modal.classList.add('show');
    
    refreshLogs();
}

function closeLogModal() {
    const modal = document.getElementById('log-modal');
    modal.classList.remove('show');
    currentLogProcess = null;
}

async function refreshLogs() {
    if (!currentLogProcess) return;
    
    const content = document.getElementById('log-modal-content');
    const result = await fetchLogs(currentLogProcess);
    
    if (result.success) {
        content.textContent = result.logs || 'No logs available';
        // Auto-scroll to bottom
        content.parentElement.scrollTop = content.parentElement.scrollHeight;
    } else {
        content.textContent = `Error: ${result.message || 'Failed to fetch logs'}`;
    }
}

// Close modal on overlay click
document.addEventListener('click', (e) => {
    if (e.target.id === 'log-modal') {
        closeLogModal();
    }
});

// Close modal on Escape key
document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
        closeLogModal();
    }
});

// ============================================
// UI Rendering Functions
// ============================================

function renderProcesses(processes) {
    const container = document.getElementById('processes-container');
    
    if (!processes || Object.keys(processes).length === 0) {
        container.innerHTML = '<div class="loading">No processes configured</div>';
        return;
    }
    
    let html = '<div class="process-list">';
    
    for (const [name, info] of Object.entries(processes)) {
        const statusClass = info.active ? 'active' : '';
        const statusIcon = info.active ? '●' : '○';
        const statusTooltip = info.active 
            ? `${name} is running` 
            : `${name} is not running`;
        
        // Determine button states and labels
        const canStop = info.can_relaunch && info.active;
        const canStart = info.can_relaunch && !info.active;
        const startTitle = info.active ? `Restart ${name}` : `Start ${name}`;
        const startIcon = info.active ? '🔄' : '▶️';
        
        // Use description for tooltip, fallback to name
        const nameTooltip = info.description || name;
        
        html += `
            <div class="process-item ${statusClass}">
                <div class="process-info">
                    <span class="process-status ${statusClass}" title="${statusTooltip}">${statusIcon}</span>
                    <span class="process-name" title="${nameTooltip}">${name}</span>
                </div>
                <div class="process-actions">
                    <button class="btn btn-logs" 
                            onclick="showLogModal('${name}')"
                            title="View logs for ${name}"
                            ${!info.can_relaunch ? 'disabled' : ''}>
                        📋
                    </button>
                    <button class="btn btn-stop" 
                            onclick="handleStop('${name}', this)"
                            title="Stop ${name} (SIGINT)"
                            ${!canStop ? 'disabled' : ''}>
                        ⏹
                    </button>
                    <button class="btn btn-relaunch ${info.active ? '' : 'btn-start'}" 
                            onclick="handleRelaunch('${name}', this)"
                            title="${startTitle}"
                            ${!info.can_relaunch ? 'disabled' : ''}>
                        ${startIcon}
                    </button>
                </div>
            </div>
        `;
    }
    
    html += '</div>';
    container.innerHTML = html;
}

function renderTopics(topics, group) {
    const containerId = `${group}-container`;
    const container = document.getElementById(containerId);
    
    if (!container) {
        console.warn(`Container not found for group: ${group}`);
        return;
    }
    
    if (!topics || Object.keys(topics).length === 0) {
        container.innerHTML = '<div class="loading">No topics configured</div>';
        return;
    }
    
    let html = '';
    
    for (const [name, info] of Object.entries(topics)) {
        const isActive = info.active;
        const activeClass = isActive ? 'active' : '';
        const statusText = isActive ? '●' : '○';
        
        // Build extra fields HTML if present
        let extraHtml = '';
        const extraFields = Object.entries(info).filter(([key]) => 
            !['name', 'topic', 'show', 'active', 'value', 'toggleable'].includes(key)
        );
        
        if (extraFields.length > 0) {
            extraHtml = '<div class="topic-extra">';
            for (const [key, value] of extraFields) {
                // Each field on its own line for readability
                extraHtml += `<div class="topic-extra-item"><span class="extra-label">${key}:</span> <span class="extra-value">${value}</span></div>`;
            }
            extraHtml += '</div>';
        }
        
        // Check if this is a boolean toggle topic (value is True/False or true/false)
        const isBoolTopic = info.value === 'True' || info.value === 'False' || 
                           info.value === true || info.value === false;
        const boolValue = info.value === 'True' || info.value === true;
        
        // Build value/toggle HTML
        let valueHtml = '';
        if (isBoolTopic) {
            const toggleClass = boolValue ? 'on' : 'off';
            valueHtml = `
                <div class="topic-toggle ${toggleClass}" onclick="handleToggle('${group}', '${name}')">
                    <span class="toggle-label">${boolValue ? 'ON' : 'OFF'}</span>
                    <span class="toggle-switch"></span>
                </div>
            `;
        } else {
            valueHtml = `<div class="topic-value">${info.value}</div>`;
        }
        
        html += `
            <div class="topic-item ${activeClass}">
                <div class="topic-header">
                    <span class="topic-status">${statusText}</span>
                    <span class="topic-name">${name}</span>
                </div>
                <div class="topic-path">${info.topic}</div>
                ${valueHtml}
                ${extraHtml}
            </div>
        `;
    }
    
    container.innerHTML = html;
}

function renderAllTopics(topicsData) {
    for (const [group, topics] of Object.entries(topicsData)) {
        renderTopics(topics, group);
    }
}

// Network traffic state for calculating speed
let lastNetworkData = {};
let lastNetworkTime = Date.now();
const MAX_SPEED = 100 * 1024 * 1024; // 100 MB/s for bar scaling

function formatBytes(bytes) {
    if (bytes < 1024) return bytes.toFixed(0) + ' B/s';
    if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB/s';
    return (bytes / (1024 * 1024)).toFixed(2) + ' MB/s';
}

function renderNetwork(networkData) {
    const container = document.getElementById('network-container');
    if (!container || !networkData) return;
    
    const currentTime = Date.now();
    const timeDiff = (currentTime - lastNetworkTime) / 1000; // seconds
    
    let html = '<div class="network-list">';
    
    for (const [iface, info] of Object.entries(networkData)) {
        const activeClass = info.active ? 'active' : '';
        
        // Calculate speed
        let rxSpeed = 0, txSpeed = 0;
        if (lastNetworkData[iface] && timeDiff > 0) {
            rxSpeed = Math.max(0, (info.rx_bytes - lastNetworkData[iface].rx_bytes) / timeDiff);
            txSpeed = Math.max(0, (info.tx_bytes - lastNetworkData[iface].tx_bytes) / timeDiff);
        }
        
        // Calculate bar percentages (log scale for better visualization)
        const txPercent = txSpeed > 0 ? Math.min(100, (Math.log10(txSpeed + 1) / Math.log10(MAX_SPEED)) * 100) : 0;
        const rxPercent = rxSpeed > 0 ? Math.min(100, (Math.log10(rxSpeed + 1) / Math.log10(MAX_SPEED)) * 100) : 0;
        
        html += `
            <div class="network-item ${activeClass}">
                <div class="network-header">
                    <div class="network-iface">
                        <span class="network-dot"></span>
                        <span class="network-name">${iface}:</span>
                        <span class="network-label">${info.label}</span>
                    </div>
                </div>
                <div class="network-bars">
                    <div class="network-bar-row">
                        <span class="network-bar-label upload">↑</span>
                        <div class="network-bar-container">
                            <div class="network-bar upload" style="width: ${txPercent}%"></div>
                        </div>
                        <span class="network-bar-value">${formatBytes(txSpeed)}</span>
                    </div>
                    <div class="network-bar-row">
                        <span class="network-bar-label download">↓</span>
                        <div class="network-bar-container">
                            <div class="network-bar download" style="width: ${rxPercent}%"></div>
                        </div>
                        <span class="network-bar-value">${formatBytes(rxSpeed)}</span>
                    </div>
                </div>
            </div>
        `;
    }
    
    html += '</div>';
    container.innerHTML = html;
    
    // Store for next calculation
    lastNetworkData = networkData;
    lastNetworkTime = currentTime;
}

function renderSystemInfo(systemData) {
    const container = document.getElementById('system-container');
    if (!container || !systemData) return;
    
    let html = '<div class="system-info-list">';
    
    // Environment variables
    for (const [label, value] of Object.entries(systemData.env || {})) {
        html += `
            <div class="system-info-item">
                <span class="system-info-label">${label}</span>
                <span class="system-info-value">${value}</span>
            </div>
        `;
    }
    
    // PTP Status as an integrated item
    const ptpActive = systemData.ptp_active;
    const ptpDotClass = ptpActive ? 'active' : 'inactive';
    const ptpText = ptpActive ? 'Active' : 'Disabled';
    const ptpValueClass = ptpActive ? '' : 'inactive';
    
    html += `
        <div class="ptp-item">
            <span class="system-info-label"><span class="ptp-dot ${ptpDotClass}"></span>PTP Master</span>
            <span class="ptp-value ${ptpValueClass}">${ptpText}</span>
        </div>
    `;
    
    // PTP Offsets if available
    if (ptpActive && systemData.ptp_offsets) {
        for (const [iface, offset] of Object.entries(systemData.ptp_offsets)) {
            html += `
                <div class="ptp-item">
                    <span class="system-info-label" style="padding-left: 12px;">${iface}</span>
                    <span class="ptp-value">${offset}</span>
                </div>
            `;
        }
    }
    
    html += '</div>';
    container.innerHTML = html;
}

// ============================================
// Event Handlers
// ============================================

async function handleRelaunch(name, button) {
    // Show loading state
    button.classList.add('loading');
    const originalContent = button.innerHTML;
    button.innerHTML = '<span class="spinner"></span>';
    button.disabled = true;
    
    // Call API
    await relaunchProcess(name);
    
    // Restore button
    setTimeout(() => {
        button.classList.remove('loading');
        button.innerHTML = originalContent;
        button.disabled = false;
    }, 1000);
}

async function handleStop(name, button) {
    // Confirm stop action
    if (!confirm(`Are you sure you want to stop "${name}"?`)) {
        return;
    }
    
    // Show loading state
    button.classList.add('loading');
    const originalContent = button.innerHTML;
    button.innerHTML = '<span class="spinner"></span>';
    button.disabled = true;
    
    // Call API
    await stopProcess(name);
    
    // Restore button (will be updated by next status poll)
    setTimeout(() => {
        button.classList.remove('loading');
        button.innerHTML = originalContent;
        // Don't re-enable - will be updated by renderProcesses based on active state
    }, 1000);
    
    // Force immediate dashboard update
    await updateDashboard();
}

// ============================================
// Masonry Layout with Horizontal Order
// ============================================

/**
 * SECTION ORDER CONFIG - Edit this array to change the order
 * Order is left-to-right, top-to-bottom (horizontal first)
 */
const SECTION_ORDER = [
    'processes',  // 1st row, 1st col
    'sensors',    // 1st row, 2nd col
    'cameras',    // 1st row, 3rd col
    'fusion',     // 2nd row, 1st col
    'network',    // 2nd row, 2nd col
    'system'      // 2nd row, 3rd col
];

/**
 * Reorder cards for CSS columns to achieve horizontal-first flow.
 * CSS columns fill vertically, so we reorder DOM elements.
 */
function reorderCardsForMasonry() {
    const container = document.getElementById('main-container');
    if (!container) return;
    
    // Get current column count from computed style
    const style = getComputedStyle(container);
    const columnCount = parseInt(style.columnCount) || 3;
    
    // Get all cards and sort by SECTION_ORDER
    const cards = Array.from(container.querySelectorAll('.card[data-section]'));
    cards.sort((a, b) => {
        const orderA = SECTION_ORDER.indexOf(a.dataset.section);
        const orderB = SECTION_ORDER.indexOf(b.dataset.section);
        return orderA - orderB;
    });
    
    const rowCount = Math.ceil(cards.length / columnCount);
    
    // Reorder: convert row-first order to column-first for CSS columns
    // CSS columns fills: col1 top-to-bottom, then col2, etc.
    // We want visual: row1 left-to-right, then row2, etc.
    const reordered = [];
    for (let col = 0; col < columnCount; col++) {
        for (let row = 0; row < rowCount; row++) {
            const index = row * columnCount + col;
            if (index < cards.length) {
                reordered.push(cards[index]);
            }
        }
    }
    
    // Re-append in new order
    reordered.forEach(card => container.appendChild(card));
}

// Reorder on resize (debounced)
let resizeTimer;
window.addEventListener('resize', () => {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(reorderCardsForMasonry, 150);
});

// ============================================
// Main Update Loop
// ============================================

async function updateDashboard() {
    const data = await fetchStatus();
    
    if (data) {
        renderProcesses(data.processes);
        renderAllTopics(data.topics);
        renderNetwork(data.network);
        renderSystemInfo(data.system);
    }
}

function startUpdateLoop() {
    // Reorder cards for masonry horizontal flow
    reorderCardsForMasonry();
    
    // Initial update
    updateDashboard();
    updateDateTime();
    
    // Set up periodic updates
    updateTimer = setInterval(updateDashboard, CONFIG.updateInterval);
    setInterval(updateDateTime, 1000);
}

// ============================================
// Initialization
// ============================================

document.addEventListener('DOMContentLoaded', () => {
    console.log('Husky Manager initializing...');
    startUpdateLoop();
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (updateTimer) {
        clearInterval(updateTimer);
    }
});

// ============================================
// Sub-GUI Navigation
// ============================================

// GUI configuration (loaded from server, with fallback defaults)
let guiConfig = {
    multiespectral: { port: 5051, path: '/', name: 'Multiespectral Camera GUI' },
    fisheye: { port: 5052, path: '/', name: 'Fisheye Camera GUI' }
};

// Load GUI config from server
async function loadGuiConfig() {
    try {
        const response = await fetchWithTimeout(`${CONFIG.apiBase}/guis`);
        if (response.ok) {
            guiConfig = await response.json();
        }
    } catch (error) {
        console.log('Using default GUI config');
    }
}

// Open a sub-GUI in a new tab
function openGui(guiName) {
    const gui = guiConfig[guiName];
    if (!gui) {
        showToast(`Unknown GUI: ${guiName}`, 'error');
        return;
    }
    
    // Build URL using current hostname with the GUI's port
    const protocol = window.location.protocol;
    const hostname = window.location.hostname;
    const url = `${protocol}//${hostname}:${gui.port}${gui.path}`;
    
    // Open in new tab
    window.open(url, `_${guiName}_gui`);
    showToast(`Opening ${gui.name}...`, 'info');
}

// Load GUI config on startup
loadGuiConfig();
