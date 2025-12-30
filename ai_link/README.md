# AI-Link

Secure communication module for offloading complex AI tasks from the RoverBot to a remote Ollama server over encrypted WiFi.

## Architecture

```
+-------------------+          TLS/HTTPS           +-------------------+
|    RoverBot       |  =========================>  |   MacBook Pro     |
|    (Pi 5)         |         WiFi LAN             |   M3 Max 36GB     |
|                   |                              |                   |
|  +-------------+  |                              |  +-------------+  |
|  | AI-Link     |  |   Encrypted JSON-RPC         |  | Ollama      |  |
|  | Client      |--+----------------------------->|  | Server      |  |
|  +-------------+  |                              |  +-------------+  |
|        |          |                              |        |          |
|  +-------------+  |                              |  +-------------+  |
|  | ROS2 Node   |  |                              |  | qwen2.5:32b |  |
|  +-------------+  |                              |  +-------------+  |
+-------------------+                              +-------------------+
```

## Features

- TLS encryption with optional mutual authentication
- Async client for non-blocking robot operation
- Connection pooling and automatic retry
- Task-specific model configurations
- ROS2 integration node
- CLI for testing and debugging

## Supported Tasks

| Task | Description | Default Model |
|------|-------------|---------------|
| Scene Analysis | Analyze camera/sensor data | qwen2.5:32b |
| Path Planning | Complex navigation decisions | qwen2.5:32b |
| Natural Language | Parse voice commands | qwen2.5:14b |
| Object ID | Identify objects | qwen2.5:14b |
| Decision Making | Mission-critical choices | qwen2.5:32b |

## Installation

### On Robot (Raspberry Pi 5)

```bash
# Install dependencies
pip install aiohttp

# Clone or copy ai_link module
cp -r ai_link /path/to/ros2_ws/src/roverbot/

# For ROS2 integration
cd /path/to/ros2_ws
colcon build --packages-select roverbot
```

### On Server (MacBook Pro M3 Max)

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull recommended models
ollama pull qwen2.5:32b-instruct-q4_K_M
ollama pull qwen2.5:14b-instruct

# Start Ollama (runs on port 11434)
ollama serve
```

## Configuration

### Basic Setup (No TLS)

```python
from ai_link import AILinkClient, AILinkConfig

config = AILinkConfig(
    host="192.168.1.100",  # MacBook IP
    port=11434,
    use_tls=False,  # For initial testing
)

async with AILinkClient(config) as client:
    response = await client.generate("Hello, robot!")
    print(response.text)
```

### Secure Setup (TLS)

```bash
# Generate certificates
cd ai_link
chmod +x setup_tls.sh
SERVER_IP=192.168.1.100 ./setup_tls.sh
```

```python
config = AILinkConfig(
    host="192.168.1.100",
    port=11434,
    use_tls=True,
    ca_cert="/path/to/certs/ca.crt",
    client_cert="/path/to/certs/client.crt",
    client_key="/path/to/certs/client.key",
)
```

### Environment Variables

```bash
export AILINK_HOST=192.168.1.100
export AILINK_PORT=11434
export AILINK_USE_TLS=true
export AILINK_MODEL=qwen2.5:32b-instruct-q4_K_M
export AILINK_CA_CERT=/path/to/ca.crt
export AILINK_CLIENT_CERT=/path/to/client.crt
export AILINK_CLIENT_KEY=/path/to/client.key
```

```python
config = AILinkConfig.from_env()
```

## Usage Examples

### Scene Analysis

```python
async with AILinkClient(config) as client:
    # With sensor description
    response = await client.analyze_scene(
        "LIDAR shows obstacle 2m ahead, 30cm wide. "
        "Camera shows cardboard box on floor."
    )
    print(response.text)

    # With image
    with open("camera.jpg", "rb") as f:
        image_data = f.read()
    response = await client.analyze_scene(
        "What's in front of the robot?",
        image=image_data
    )
```

### Natural Language Commands

```python
response = await client.process_command(
    "Go to the kitchen and find the red cup"
)
# Returns structured JSON:
# {
#   "intent": "fetch object",
#   "action": "navigate",
#   "parameters": {"location": "kitchen", "target": "red cup"},
#   "confidence": 0.92
# }
```

### Path Planning Assistance

```python
response = await client.plan_path(
    current_position={"x": 0, "y": 0, "theta": 0},
    goal_position={"x": 5, "y": 3, "theta": 1.57},
    obstacles=[
        {"x": 2, "y": 1, "radius": 0.5},
        {"x": 3, "y": 2, "radius": 0.3},
    ],
    constraints={"max_velocity": 0.5, "min_clearance": 0.2}
)
```

### Decision Making

```python
response = await client.make_decision(
    situation="Battery at 15%, 200m from charging station, mission incomplete",
    options=[
        "Continue mission",
        "Return to charge",
        "Enter low-power mode and wait"
    ],
    context={"mission_priority": "medium", "weather": "rain_expected"}
)
```

## ROS2 Integration

### Launch Node

```bash
ros2 run roverbot ai_link_node --ros-args \
    -p host:=192.168.1.100 \
    -p port:=11434 \
    -p use_tls:=true \
    -p model:=qwen2.5:32b-instruct-q4_K_M
```

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/ai_link/command` | String | Sub | NL commands |
| `/ai_link/response` | String | Pub | AI responses (JSON) |
| `/ai_link/status` | String | Pub | Connection status |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/ai_link/health_check` | Trigger | Check connection |

### Example Usage

```bash
# Check status
ros2 service call /ai_link/health_check std_srvs/srv/Trigger

# Send command
ros2 topic pub /ai_link/command std_msgs/String "data: 'go to the door'"

# Listen for responses
ros2 topic echo /ai_link/response
```

## CLI Testing

```bash
# Check connection
python -m ai_link.cli --host 192.168.1.100 status

# List models
python -m ai_link.cli --host 192.168.1.100 models

# Generate text
python -m ai_link.cli --host 192.168.1.100 generate "What is 2+2?"

# Process command
python -m ai_link.cli --host 192.168.1.100 command "find the nearest chair"

# Analyze scene
python -m ai_link.cli --host 192.168.1.100 scene "obstacle detected 1m ahead"

# Generate config file
python -m ai_link.cli init-config config.json
```

## Server Setup (HTTPS with nginx)

For production, use nginx as a TLS termination proxy:

```nginx
# /etc/nginx/sites-available/ollama
server {
    listen 11434 ssl;
    server_name gpu-server;

    ssl_certificate     /path/to/server.crt;
    ssl_certificate_key /path/to/server.key;
    ssl_client_certificate /path/to/ca.crt;
    ssl_verify_client optional;

    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_ciphers HIGH:!aNULL:!MD5;

    location / {
        proxy_pass http://127.0.0.1:11435;  # Ollama on different port
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_read_timeout 300s;
    }
}
```

Then run Ollama on localhost:11435:

```bash
OLLAMA_HOST=127.0.0.1:11435 ollama serve
```

## Model Recommendations (M3 Max 36GB)

| Model | VRAM | Speed | Best For |
|-------|------|-------|----------|
| qwen2.5:32b-instruct-q4_K_M | ~20GB | Good | Complex reasoning, planning |
| qwen2.5:14b-instruct | ~9GB | Fast | Quick responses, simple tasks |
| deepseek-r1:32b | ~20GB | Good | Mathematical/logical reasoning |
| llava:34b | ~22GB | Good | Vision/multimodal (with images) |

## Troubleshooting

### Connection Refused

```bash
# Check Ollama is running
curl http://192.168.1.100:11434/api/tags

# Check firewall
sudo ufw allow 11434/tcp
```

### SSL Certificate Errors

```bash
# Verify certificates
openssl verify -CAfile ca.crt server.crt
openssl s_client -connect 192.168.1.100:11434 -CAfile ca.crt
```

### Slow Responses

- Use smaller model (qwen2.5:14b) for time-sensitive tasks
- Reduce `max_tokens` in config
- Check WiFi signal strength

## Files

```
ai_link/
├── __init__.py       # Package init
├── client.py         # Main async client
├── config.py         # Configuration
├── ros2_node.py      # ROS2 integration
├── cli.py            # Command-line interface
├── setup_tls.sh      # TLS certificate generator
├── requirements.txt  # Python dependencies
└── README.md         # This file
```
