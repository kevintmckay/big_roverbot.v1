#!/usr/bin/env python3
"""
AI-Link ROS2 Node: Integrates AI-Link with ROS2 for Big RoverBot.

Provides ROS2 services and topics for offloading AI tasks to the
remote GPU server running Ollama.
"""

import asyncio
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

# Custom message/service types would be defined in a ROS2 package
# For now, using String for flexibility

from .client import AILinkClient, TaskType, AIResponse
from .config import AILinkConfig


class AILinkNode(Node):
    """
    ROS2 Node for AI-Link integration.

    Subscriptions:
        /ai_link/command (String): Natural language commands
        /camera/image_raw (Image): Camera feed for visual analysis

    Publishers:
        /ai_link/response (String): AI responses (JSON)
        /ai_link/status (String): Connection status

    Services:
        /ai_link/analyze_scene: Analyze current scene
        /ai_link/process_command: Process NL command
        /ai_link/make_decision: Get decision support
        /ai_link/health_check: Check connection status
    """

    def __init__(self):
        super().__init__("ai_link_node")

        # Declare parameters
        self.declare_parameter("host", "192.168.1.100")
        self.declare_parameter("port", 11434)
        self.declare_parameter("use_tls", True)
        self.declare_parameter("verify_ssl", True)
        self.declare_parameter("model", "qwen2.5:32b")
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("ca_cert", "")
        self.declare_parameter("client_cert", "")
        self.declare_parameter("client_key", "")

        # Build config from parameters
        self.config = AILinkConfig(
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
            use_tls=self.get_parameter("use_tls").value,
            verify_ssl=self.get_parameter("verify_ssl").value,
            default_model=self.get_parameter("model").value,
            timeout=self.get_parameter("timeout").value,
            ca_cert=self.get_parameter("ca_cert").value or None,
            client_cert=self.get_parameter("client_cert").value or None,
            client_key=self.get_parameter("client_key").value or None,
        )

        # Initialize client
        self.client = AILinkClient(self.config)
        self._loop = asyncio.new_event_loop()
        self._connected = False

        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Publishers
        self.response_pub = self.create_publisher(
            String, "/ai_link/response", 10
        )
        self.status_pub = self.create_publisher(
            String, "/ai_link/status", 10
        )

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            "/ai_link/command",
            self._command_callback,
            10,
            callback_group=self.callback_group,
        )

        # Store latest camera image
        self._latest_image: Optional[bytes] = None
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self._image_callback,
            1,
            callback_group=self.callback_group,
        )

        # Services
        self.health_srv = self.create_service(
            Trigger,
            "/ai_link/health_check",
            self._health_check_callback,
            callback_group=self.callback_group,
        )

        # Status timer
        self.status_timer = self.create_timer(
            5.0, self._publish_status, callback_group=self.callback_group
        )

        # Connect on startup
        self.startup_timer = self.create_timer(
            1.0, self._startup_connect, callback_group=self.callback_group
        )

        self.get_logger().info("AI-Link node initialized")

    def _run_async(self, coro):
        """Run async coroutine in the event loop."""
        return self._loop.run_until_complete(coro)

    async def _startup_connect_async(self):
        """Async startup connection."""
        try:
            success = await self.client.connect()
            if success:
                self._connected = True
                self.get_logger().info(
                    f"Connected to Ollama at {self.config.base_url}"
                )
                models = await self.client.list_models()
                self.get_logger().info(f"Available models: {models}")
            else:
                self.get_logger().warn("Failed to connect to Ollama server")
        except Exception as e:
            self.get_logger().error(f"Connection error: {e}")

    def _startup_connect(self):
        """Initial connection attempt."""
        self._run_async(self._startup_connect_async())
        # Cancel after first run
        self.startup_timer.cancel()

    def _publish_status(self):
        """Publish connection status."""
        msg = String()
        msg.data = json.dumps({
            "connected": self._connected,
            "host": self.config.host,
            "port": self.config.port,
            "model": self.config.default_model,
        })
        self.status_pub.publish(msg)

    def _image_callback(self, msg: Image):
        """Store latest camera image."""
        # Convert ROS Image to bytes (simplified - actual implementation
        # would properly decode based on encoding)
        self._latest_image = bytes(msg.data)

    async def _process_command_async(self, command: str) -> AIResponse:
        """Process a natural language command."""
        return await self.client.process_command(command)

    def _command_callback(self, msg: String):
        """Handle incoming natural language commands."""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        if not self._connected:
            self.get_logger().warn("Not connected to AI server")
            return

        try:
            response = self._run_async(self._process_command_async(command))

            # Publish response
            resp_msg = String()
            resp_msg.data = json.dumps(response.to_dict())
            self.response_pub.publish(resp_msg)

            self.get_logger().info(f"AI response: {response.text[:100]}...")

        except Exception as e:
            self.get_logger().error(f"Command processing error: {e}")

    def _health_check_callback(self, request, response):
        """Health check service callback."""
        if self._connected:
            response.success = True
            response.message = f"Connected to {self.config.base_url}"
        else:
            response.success = False
            response.message = "Not connected to AI server"
        return response

    def destroy_node(self):
        """Cleanup on shutdown."""
        self._run_async(self.client.disconnect())
        self._loop.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = AILinkNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
