#pragma once

static const char WEBPROV_HTML_TEMPLATE[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Device Setup</title>
    <style>
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            background: linear-gradient(135deg, #667eea 0%%, #764ba2 100%%);
            min-height: 100vh;
            padding: 20px;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        .container {
            background: white;
            border-radius: 16px;
            padding: 32px 24px;
            width: 100%%;
            max-width: 400px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.2);
        }
        h1 {
            color: #333;
            font-size: 24px;
            text-align: center;
            margin-bottom: 8px;
        }
        .subtitle {
            color: #666;
            text-align: center;
            font-size: 14px;
            margin-bottom: 24px;
        }
        .section {
            margin-bottom: 24px;
        }
        .section-title {
            color: #667eea;
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 12px;
        }
        .form-group {
            margin-bottom: 16px;
        }
        label {
            display: block;
            color: #333;
            font-size: 14px;
            font-weight: 500;
            margin-bottom: 6px;
        }
        input {
            width: 100%%;
            padding: 14px 16px;
            border: 2px solid #e1e1e1;
            border-radius: 10px;
            font-size: 16px;
            transition: border-color 0.2s, box-shadow 0.2s;
            -webkit-appearance: none;
        }
        input:focus {
            outline: none;
            border-color: #667eea;
            box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
        }
        input.error {
            border-color: #e74c3c;
        }
        .error-message {
            color: #e74c3c;
            font-size: 12px;
            margin-top: 4px;
            display: none;
        }
        .error-message.visible {
            display: block;
        }
        .alert {
            background: #fee2e2;
            border: 1px solid #fecaca;
            color: #dc2626;
            padding: 12px 16px;
            border-radius: 10px;
            margin-bottom: 20px;
            font-size: 14px;
            display: none;
        }
        .alert.visible {
            display: block;
        }
        .buttons {
            display: flex;
            gap: 12px;
            margin-top: 24px;
        }
        button {
            flex: 1;
            padding: 16px 24px;
            border: none;
            border-radius: 10px;
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
            transition: transform 0.1s, box-shadow 0.2s;
        }
        button:active {
            transform: scale(0.98);
        }
        .btn-primary {
            background: linear-gradient(135deg, #667eea 0%%, #764ba2 100%%);
            color: white;
            box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
        }
        .btn-primary:hover {
            box-shadow: 0 6px 20px rgba(102, 126, 234, 0.5);
        }
        .btn-secondary {
            background: #f3f4f6;
            color: #374151;
        }
        .btn-secondary:hover {
            background: #e5e7eb;
        }
        .spinner {
            display: none;
            width: 20px;
            height: 20px;
            border: 2px solid #ffffff;
            border-top-color: transparent;
            border-radius: 50%%;
            animation: spin 0.8s linear infinite;
            margin: 0 auto;
        }
        @keyframes spin {
            to { transform: rotate(360deg); }
        }
        .btn-primary.loading .spinner {
            display: inline-block;
        }
        .btn-primary.loading .btn-text {
            display: none;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Device Setup</h1>
        <p class="subtitle">Configure WiFi and MQTT settings</p>

        <div id="alert" class="alert">%s</div>

        <form id="configForm" novalidate>
            <div class="section">
                <div class="section-title">WiFi Settings</div>
                <div class="form-group">
                    <label for="wifi_ssid">Network Name (SSID)</label>
                    <input type="text" id="wifi_ssid" name="wifi_ssid" value="%s" required autocomplete="off" autocapitalize="none">
                    <div class="error-message" id="wifi_ssid-error">Please enter the WiFi network name</div>
                </div>
                <div class="form-group">
                    <label for="wifi_password">WiFi Password</label>
                    <input type="password" id="wifi_password" name="wifi_password" value="%s" required>
                    <div class="error-message" id="wifi_password-error">Please enter the WiFi password</div>
                </div>
            </div>

            <div class="section">
                <div class="section-title">MQTT Settings</div>
                <div class="form-group">
                    <label for="mqtt_broker">MQTT Broker URL</label>
                    <input type="text" id="mqtt_broker" name="mqtt_broker" value="%s" required placeholder="mqtt://broker.example.com:1883" autocomplete="off" autocapitalize="none">
                    <div class="error-message" id="mqtt_broker-error">Please enter the MQTT broker URL</div>
                </div>
                <div class="form-group">
                    <label for="mqtt_user">MQTT Username</label>
                    <input type="text" id="mqtt_user" name="mqtt_user" value="%s" required autocomplete="off" autocapitalize="none">
                    <div class="error-message" id="mqtt_user-error">Please enter the MQTT username</div>
                </div>
                <div class="form-group">
                    <label for="mqtt_password">MQTT Password</label>
                    <input type="password" id="mqtt_password" name="mqtt_password" value="%s" required>
                    <div class="error-message" id="mqtt_password-error">Please enter the MQTT password</div>
                </div>
            </div>

            <div class="buttons">
                <button type="button" class="btn-secondary" onclick="cancel()">Cancel</button>
                <button type="submit" class="btn-primary" id="submitBtn">
                    <span class="btn-text">Save</span>
                    <div class="spinner"></div>
                </button>
            </div>
        </form>
    </div>

    <script>
        const form = document.getElementById('configForm');
        const submitBtn = document.getElementById('submitBtn');
        const alert = document.getElementById('alert');
        const fields = ['wifi_ssid', 'wifi_password', 'mqtt_broker', 'mqtt_user', 'mqtt_password'];

        function validateField(name) {
            const input = document.getElementById(name);
            const error = document.getElementById(name + '-error');
            const isValid = input.value.trim() !== '';
            input.classList.toggle('error', !isValid);
            error.classList.toggle('visible', !isValid);
            return isValid;
        }

        function validateAll() {
            let valid = true;
            fields.forEach(f => {
                if (!validateField(f)) valid = false;
            });
            return valid;
        }

        fields.forEach(f => {
            document.getElementById(f).addEventListener('blur', () => validateField(f));
            document.getElementById(f).addEventListener('input', () => {
                const input = document.getElementById(f);
                if (input.classList.contains('error')) validateField(f);
            });
        });

        form.addEventListener('submit', async (e) => {
            e.preventDefault();
            alert.classList.remove('visible');

            if (!validateAll()) return;

            submitBtn.classList.add('loading');
            submitBtn.disabled = true;

            const data = {};
            fields.forEach(f => data[f] = document.getElementById(f).value);

            try {
                const res = await fetch('/config', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });

                if (res.ok) {
                    alert.textContent = 'Configuration saved! Device is rebooting...';
                    alert.style.background = '#d1fae5';
                    alert.style.borderColor = '#a7f3d0';
                    alert.style.color = '#065f46';
                    alert.classList.add('visible');
                } else {
                    const err = await res.text();
                    throw new Error(err || 'Configuration failed');
                }
            } catch (err) {
                alert.textContent = err.message || 'Failed to save configuration. Please check your input.';
                alert.classList.add('visible');
                submitBtn.classList.remove('loading');
                submitBtn.disabled = false;
            }
        });

        function cancel() {
            fetch('/cancel', {method: 'POST'}).then(() => {
                alert.textContent = 'Cancelled. Device is rebooting...';
                alert.style.background = '#fef3c7';
                alert.style.borderColor = '#fde68a';
                alert.style.color = '#92400e';
                alert.classList.add('visible');
            });
        }
    </script>
</body>
</html>
)rawliteral";
