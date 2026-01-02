#!/bin/bash
#
# AI-Link TLS Setup Script
# Generates self-signed certificates for encrypted communication
# between the robot and the GPU server running Ollama.
#
# Usage:
#   ./setup_tls.sh [output_dir]
#
# This creates:
#   - CA certificate and key (for signing)
#   - Server certificate and key (for Ollama server)
#   - Client certificate and key (for robot/mutual TLS)
#

set -e

OUTPUT_DIR="${1:-./certs}"
DAYS_VALID=365
KEY_SIZE=4096

# Server details (customize these)
SERVER_IP="${SERVER_IP:-192.168.1.100}"
SERVER_HOSTNAME="${SERVER_HOSTNAME:-gpu-server}"
ROBOT_HOSTNAME="${ROBOT_HOSTNAME:-big_roverbot}"

echo "=== AI-Link TLS Certificate Setup ==="
echo "Output directory: $OUTPUT_DIR"
echo "Server IP: $SERVER_IP"
echo "Server hostname: $SERVER_HOSTNAME"
echo "Robot hostname: $ROBOT_HOSTNAME"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR"

# ============================================================================
# 1. Generate CA (Certificate Authority)
# ============================================================================
echo "[1/4] Generating CA certificate..."

openssl genrsa -out ca.key $KEY_SIZE

openssl req -new -x509 -days $DAYS_VALID -key ca.key -out ca.crt \
    -subj "/C=US/ST=Local/L=Local/O=RoverBot/OU=AI-Link/CN=AI-Link-CA"

echo "  Created: ca.key, ca.crt"

# ============================================================================
# 2. Generate Server Certificate (for Ollama GPU server)
# ============================================================================
echo "[2/4] Generating server certificate..."

openssl genrsa -out server.key $KEY_SIZE

# Create config for SAN (Subject Alternative Names)
cat > server.cnf << EOF
[req]
default_bits = $KEY_SIZE
prompt = no
default_md = sha256
req_extensions = req_ext
distinguished_name = dn

[dn]
C = US
ST = Local
L = Local
O = RoverBot
OU = AI-Link
CN = $SERVER_HOSTNAME

[req_ext]
subjectAltName = @alt_names

[alt_names]
DNS.1 = $SERVER_HOSTNAME
DNS.2 = localhost
IP.1 = $SERVER_IP
IP.2 = 127.0.0.1
EOF

openssl req -new -key server.key -out server.csr -config server.cnf

openssl x509 -req -days $DAYS_VALID \
    -in server.csr \
    -CA ca.crt -CAkey ca.key -CAcreateserial \
    -out server.crt \
    -extfile server.cnf -extensions req_ext

rm server.csr server.cnf

echo "  Created: server.key, server.crt"

# ============================================================================
# 3. Generate Client Certificate (for robot - mutual TLS)
# ============================================================================
echo "[3/4] Generating client certificate..."

openssl genrsa -out client.key $KEY_SIZE

openssl req -new -key client.key -out client.csr \
    -subj "/C=US/ST=Local/L=Local/O=RoverBot/OU=AI-Link/CN=$ROBOT_HOSTNAME"

openssl x509 -req -days $DAYS_VALID \
    -in client.csr \
    -CA ca.crt -CAkey ca.key -CAcreateserial \
    -out client.crt

rm client.csr

echo "  Created: client.key, client.crt"

# ============================================================================
# 4. Create combined files and set permissions
# ============================================================================
echo "[4/4] Setting permissions and creating bundles..."

# Restrict key file permissions
chmod 600 *.key
chmod 644 *.crt

# Create combined PEM files (useful for some applications)
cat server.crt server.key > server.pem
cat client.crt client.key > client.pem
chmod 600 *.pem

echo "  Permissions set"

# ============================================================================
# Summary
# ============================================================================
echo ""
echo "=== Setup Complete ==="
echo ""
echo "Files created in $OUTPUT_DIR:"
echo "  CA:     ca.crt, ca.key"
echo "  Server: server.crt, server.key, server.pem"
echo "  Client: client.crt, client.key, client.pem"
echo ""
echo "=== Installation Instructions ==="
echo ""
echo "ON GPU SERVER (Ollama):"
echo "  1. Copy server.crt, server.key, and ca.crt to the server"
echo "  2. Configure Ollama to use HTTPS (via reverse proxy like nginx/caddy)"
echo "  3. Example nginx config:"
echo ""
echo "     server {"
echo "         listen 11434 ssl;"
echo "         ssl_certificate     /path/to/server.crt;"
echo "         ssl_certificate_key /path/to/server.key;"
echo "         ssl_client_certificate /path/to/ca.crt;  # For mutual TLS"
echo "         ssl_verify_client optional;  # or 'on' for required"
echo ""
echo "         location / {"
echo "             proxy_pass http://localhost:11434;"
echo "         }"
echo "     }"
echo ""
echo "ON ROBOT:"
echo "  1. Copy ca.crt, client.crt, client.key to the robot"
echo "  2. Configure AI-Link:"
echo ""
echo "     config = AILinkConfig("
echo "         host='$SERVER_IP',"
echo "         port=11434,"
echo "         use_tls=True,"
echo "         ca_cert='/path/to/ca.crt',"
echo "         client_cert='/path/to/client.crt',"
echo "         client_key='/path/to/client.key',"
echo "     )"
echo ""
echo "=== Verify Certificates ==="
echo ""
echo "To verify the certificates:"
echo "  openssl verify -CAfile ca.crt server.crt"
echo "  openssl verify -CAfile ca.crt client.crt"
echo ""
