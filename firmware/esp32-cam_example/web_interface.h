#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

// Embedded HTML/CSS/JS for the web control interface
const char* WEB_INTERFACE_HTML = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Quadruped Robot Control</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }
        
        .panel {
            background: white;
            border-radius: 15px;
            padding: 25px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        
        .video-panel {
            grid-column: 1 / -1;
        }
        
        h1 {
            color: #333;
            margin-bottom: 20px;
            text-align: center;
        }
        
        h2 {
            color: #555;
            margin-bottom: 15px;
            font-size: 1.3em;
        }
        
        .video-container {
            position: relative;
            width: 100%;
            padding-bottom: 56.25%;
            background: #000;
            border-radius: 10px;
            overflow: hidden;
        }
        
        #videoStream {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            object-fit: contain;
        }
        
        .control-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }
        
        .btn {
            padding: 15px 25px;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
        }
        
        .btn-danger {
            background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
            color: white;
        }
        
        .btn-danger:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(245, 87, 108, 0.4);
        }
        
        .btn:active {
            transform: translateY(0);
        }
        
        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        
        .emergency-stop {
            grid-column: 1 / -1;
            padding: 20px;
            font-size: 20px;
            margin-top: 10px;
        }
        
        .status-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
        }
        
        .status-item {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
            border-left: 4px solid #667eea;
        }
        
        .status-label {
            font-size: 12px;
            color: #666;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 5px;
        }
        
        .status-value {
            font-size: 20px;
            font-weight: bold;
            color: #333;
        }
        
        .status-value.warning {
            color: #f39c12;
        }
        
        .status-value.danger {
            color: #e74c3c;
        }
        
        .slider-container {
            margin: 15px 0;
        }
        
        .slider-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 5px;
            color: #666;
            font-size: 14px;
        }
        
        input[type="range"] {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: #ddd;
            outline: none;
        }
        
        input[type="range"]::-webkit-slider-thumb {
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
        }
        
        .manual-control {
            margin-top: 20px;
            padding-top: 20px;
            border-top: 2px solid #eee;
        }
        
        .manual-control h3 {
            margin-bottom: 15px;
            color: #555;
        }
        
        .manual-inputs {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-bottom: 10px;
        }
        
        input[type="number"] {
            padding: 10px;
            border: 2px solid #ddd;
            border-radius: 5px;
            font-size: 14px;
        }
        
        input[type="number"]:focus {
            border-color: #667eea;
            outline: none;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="panel video-panel">
            <h1>🤖 Quadruped Robot Control</h1>
            <div class="video-container">
                <img id="videoStream" src="/stream" alt="Video Stream">
            </div>
        </div>
        
        <div class="panel">
            <h2>Gait Control</h2>
            <div class="control-grid">
                <button class="btn btn-primary" onclick="sendGait('walk')">Walk</button>
                <button class="btn btn-primary" onclick="sendGait('roll')">Roll</button>
                <button class="btn btn-primary" onclick="sendGait('climb')">Climb</button>
                <button class="btn btn-primary" onclick="sendGait('stop')">Stop</button>
            </div>
            
            <div class="slider-container">
                <div class="slider-label">
                    <span>Speed</span>
                    <span id="speedValue">0.5</span>
                </div>
                <input type="range" id="speedSlider" min="0" max="1" step="0.1" value="0.5" oninput="updateSpeed(this.value)">
            </div>
            
            <div class="slider-container">
                <div class="slider-label">
                    <span>Direction</span>
                    <span id="directionValue">0°</span>
                </div>
                <input type="range" id="directionSlider" min="-180" max="180" step="5" value="0" oninput="updateDirection(this.value)">
            </div>
            
            <button class="btn btn-danger emergency-stop" onclick="emergencyStop()">🛑 EMERGENCY STOP</button>
        </div>
        
        <div class="panel">
            <h2>Status</h2>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">State</div>
                    <div class="status-value" id="stateValue">IDLE</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Battery</div>
                    <div class="status-value" id="batteryValue">-- V</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Roll</div>
                    <div class="status-value" id="rollValue">--°</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Pitch</div>
                    <div class="status-value" id="pitchValue">--°</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Uptime</div>
                    <div class="status-value" id="uptimeValue">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Emergency Stop</div>
                    <div class="status-value" id="emergencyValue">No</div>
                </div>
            </div>
        </div>
        
        <div class="panel">
            <h2>Manual Servo Control</h2>
            <div class="manual-control">
                <h3>Calibration Mode</h3>
                <div class="manual-inputs">
                    <input type="number" id="servoLeg" placeholder="Leg (0-3)" min="0" max="3" value="0">
                    <input type="number" id="servoJoint" placeholder="Joint (0-2)" min="0" max="2" value="0">
                    <input type="number" id="servoAngle" placeholder="Angle (0-180)" min="0" max="180" value="90">
                </div>
                <button class="btn btn-primary" onclick="sendServoControl()">Set Servo</button>
            </div>
        </div>
    </div>
    
    <script>
        let currentSpeed = 0.5;
        let currentDirection = 0;
        
        function updateSpeed(value) {
            currentSpeed = parseFloat(value);
            document.getElementById('speedValue').textContent = currentSpeed.toFixed(1);
        }
        
        function updateDirection(value) {
            currentDirection = parseInt(value);
            document.getElementById('directionValue').textContent = currentDirection + '°';
        }
        
        function sendGait(gait) {
            fetch('/control', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    action: 'gait',
                    gait: gait,
                    speed: currentSpeed,
                    direction: currentDirection
                })
            })
            .then(response => response.json())
            .then(data => {
                console.log('Gait command sent:', data);
            })
            .catch(error => {
                console.error('Error:', error);
            });
        }
        
        function emergencyStop() {
            if (confirm('Activate emergency stop?')) {
                fetch('/control', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        action: 'emergency_stop'
                    })
                })
                .then(response => response.json())
                .then(data => {
                    console.log('Emergency stop activated:', data);
                })
                .catch(error => {
                    console.error('Error:', error);
                });
            }
        }
        
        function sendServoControl() {
            const leg = parseInt(document.getElementById('servoLeg').value);
            const joint = parseInt(document.getElementById('servoJoint').value);
            const angle = parseFloat(document.getElementById('servoAngle').value);
            
            fetch('/control', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    action: 'servo',
                    leg: leg,
                    joint: joint,
                    angle: angle
                })
            })
            .then(response => response.json())
            .then(data => {
                console.log('Servo command sent:', data);
            })
            .catch(error => {
                console.error('Error:', error);
            });
        }
        
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    const stateNames = ['IDLE', 'WALKING', 'ROLLING', 'CLIMBING', 'EMERGENCY_STOP', 'ERROR'];
                    document.getElementById('stateValue').textContent = stateNames[data.state] || 'UNKNOWN';
                    document.getElementById('batteryValue').textContent = data.batteryVoltage.toFixed(2) + ' V';
                    document.getElementById('rollValue').textContent = data.roll.toFixed(1) + '°';
                    document.getElementById('pitchValue').textContent = data.pitch.toFixed(1) + '°';
                    document.getElementById('uptimeValue').textContent = formatUptime(data.uptime);
                    document.getElementById('emergencyValue').textContent = data.emergencyStop ? 'YES' : 'No';
                    document.getElementById('emergencyValue').className = data.emergencyStop ? 'status-value danger' : 'status-value';
                    
                    // Update battery color
                    const batteryEl = document.getElementById('batteryValue');
                    if (data.batteryVoltage < 6.0) {
                        batteryEl.className = 'status-value danger';
                    } else if (data.batteryVoltage < 7.0) {
                        batteryEl.className = 'status-value warning';
                    } else {
                        batteryEl.className = 'status-value';
                    }
                })
                .catch(error => {
                    console.error('Error fetching status:', error);
                });
        }
        
        function formatUptime(ms) {
            const seconds = Math.floor(ms / 1000);
            const minutes = Math.floor(seconds / 60);
            const hours = Math.floor(minutes / 60);
            return hours + 'h ' + (minutes % 60) + 'm ' + (seconds % 60) + 's';
        }
        
        // Update status every second
        setInterval(updateStatus, 1000);
        updateStatus();
    </script>
</body>
</html>
)";

#endif // WEB_INTERFACE_H

