"""
AI-Link Client: Secure async client for Ollama communication.

Provides encrypted communication with a remote Ollama server
for offloading complex AI inference from the robot.
"""

import ssl
import json
import asyncio
import logging
import base64
from typing import Optional, AsyncIterator, Any
from dataclasses import dataclass
from enum import Enum

import aiohttp

from .config import AILinkConfig


logger = logging.getLogger(__name__)


class TaskType(Enum):
    """Predefined task types for robot operations."""
    SCENE_ANALYSIS = "scene_analysis"
    PATH_PLANNING = "path_planning"
    NATURAL_LANGUAGE = "natural_language"
    OBJECT_IDENTIFICATION = "object_identification"
    DECISION_MAKING = "decision_making"
    CUSTOM = "custom"


@dataclass
class AIResponse:
    """Response from the AI server."""
    text: str
    model: str
    task_type: str
    tokens_used: int
    duration_ms: float
    success: bool
    error: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "text": self.text,
            "model": self.model,
            "task_type": self.task_type,
            "tokens_used": self.tokens_used,
            "duration_ms": self.duration_ms,
            "success": self.success,
            "error": self.error,
        }


class AILinkClient:
    """
    Async client for secure communication with Ollama server.

    Features:
    - TLS encryption with optional mutual authentication
    - Connection pooling for efficiency
    - Automatic retry with exponential backoff
    - Streaming responses for long-running tasks
    - Task-specific model configurations
    """

    def __init__(self, config: Optional[AILinkConfig] = None):
        self.config = config or AILinkConfig()
        self._session: Optional[aiohttp.ClientSession] = None
        self._ssl_context: Optional[ssl.SSLContext] = None
        self._connected = False

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.disconnect()

    def _create_ssl_context(self) -> Optional[ssl.SSLContext]:
        """Create SSL context for TLS encryption."""
        if not self.config.use_tls:
            return None

        # Create SSL context with secure defaults
        context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)

        # Set minimum TLS version
        context.minimum_version = ssl.TLSVersion.TLSv1_2

        # Load CA certificate if provided
        if self.config.ca_cert:
            context.load_verify_locations(self.config.ca_cert)

        # Set verification mode
        if not self.config.verify_ssl:
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
            logger.warning("SSL verification disabled - use only for development")

        # Load client certificate for mutual TLS
        if self.config.client_cert and self.config.client_key:
            context.load_cert_chain(
                certfile=self.config.client_cert,
                keyfile=self.config.client_key,
            )
            logger.info("Mutual TLS enabled with client certificate")

        return context

    async def connect(self) -> bool:
        """Establish connection to Ollama server."""
        if self._session is not None:
            return True

        try:
            self._ssl_context = self._create_ssl_context()

            connector = aiohttp.TCPConnector(
                limit=self.config.max_connections,
                ssl=self._ssl_context,
                keepalive_timeout=30 if self.config.keepalive else 0,
            )

            timeout = aiohttp.ClientTimeout(total=self.config.timeout)

            self._session = aiohttp.ClientSession(
                connector=connector,
                timeout=timeout,
            )

            # Test connection
            if await self._health_check():
                self._connected = True
                logger.info(f"Connected to Ollama at {self.config.base_url}")
                return True
            else:
                logger.error("Health check failed")
                return False

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    async def disconnect(self) -> None:
        """Close connection to server."""
        if self._session:
            await self._session.close()
            self._session = None
            self._connected = False
            logger.info("Disconnected from Ollama server")

    async def _health_check(self) -> bool:
        """Check if server is reachable and healthy."""
        try:
            async with self._session.get(f"{self.config.base_url}/api/tags") as resp:
                return resp.status == 200
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return False

    async def list_models(self) -> list[str]:
        """List available models on the server."""
        if not self._session:
            await self.connect()

        try:
            async with self._session.get(f"{self.config.base_url}/api/tags") as resp:
                if resp.status == 200:
                    data = await resp.json()
                    return [m["name"] for m in data.get("models", [])]
        except Exception as e:
            logger.error(f"Failed to list models: {e}")
        return []

    async def _request_with_retry(
        self,
        endpoint: str,
        payload: dict,
    ) -> Optional[dict]:
        """Make request with automatic retry on failure."""
        if not self._session:
            await self.connect()

        last_error = None
        for attempt in range(self.config.max_retries):
            try:
                async with self._session.post(
                    f"{self.config.base_url}{endpoint}",
                    json=payload,
                ) as resp:
                    if resp.status == 200:
                        return await resp.json()
                    else:
                        error_text = await resp.text()
                        logger.warning(f"Request failed ({resp.status}): {error_text}")
                        last_error = error_text

            except asyncio.TimeoutError:
                logger.warning(f"Request timed out (attempt {attempt + 1})")
                last_error = "Timeout"
            except Exception as e:
                logger.warning(f"Request error (attempt {attempt + 1}): {e}")
                last_error = str(e)

            if attempt < self.config.max_retries - 1:
                delay = self.config.retry_delay * (2 ** attempt)
                await asyncio.sleep(delay)

        logger.error(f"All retry attempts failed: {last_error}")
        return None

    async def generate(
        self,
        prompt: str,
        task_type: TaskType = TaskType.CUSTOM,
        model: Optional[str] = None,
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        images: Optional[list[bytes]] = None,
    ) -> AIResponse:
        """
        Generate a response from the AI model.

        Args:
            prompt: The input prompt/question
            task_type: Type of task for config lookup
            model: Override model selection
            system_prompt: System context for the model
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            images: List of images (bytes) for multimodal tasks

        Returns:
            AIResponse with the generated text and metadata
        """
        import time
        start_time = time.time()

        # Get task-specific config
        task_config = self.config.task_configs.get(task_type.value, {})

        # Build payload
        payload = {
            "model": model or task_config.get("model", self.config.default_model),
            "prompt": prompt,
            "stream": False,
            "options": {},
        }

        if system_prompt:
            payload["system"] = system_prompt

        if temperature is not None:
            payload["options"]["temperature"] = temperature
        elif "temperature" in task_config:
            payload["options"]["temperature"] = task_config["temperature"]

        if max_tokens is not None:
            payload["options"]["num_predict"] = max_tokens
        elif "max_tokens" in task_config:
            payload["options"]["num_predict"] = task_config["max_tokens"]

        # Handle images for multimodal models
        if images:
            payload["images"] = [base64.b64encode(img).decode() for img in images]

        # Make request
        response = await self._request_with_retry("/api/generate", payload)
        duration_ms = (time.time() - start_time) * 1000

        if response:
            return AIResponse(
                text=response.get("response", ""),
                model=response.get("model", payload["model"]),
                task_type=task_type.value,
                tokens_used=response.get("eval_count", 0),
                duration_ms=duration_ms,
                success=True,
            )
        else:
            return AIResponse(
                text="",
                model=payload["model"],
                task_type=task_type.value,
                tokens_used=0,
                duration_ms=duration_ms,
                success=False,
                error="Request failed after retries",
            )

    async def generate_stream(
        self,
        prompt: str,
        task_type: TaskType = TaskType.CUSTOM,
        model: Optional[str] = None,
        system_prompt: Optional[str] = None,
    ) -> AsyncIterator[str]:
        """
        Stream response tokens from the AI model.

        Useful for long-running tasks where partial results are helpful.
        """
        if not self._session:
            await self.connect()

        task_config = self.config.task_configs.get(task_type.value, {})

        payload = {
            "model": model or task_config.get("model", self.config.default_model),
            "prompt": prompt,
            "stream": True,
        }

        if system_prompt:
            payload["system"] = system_prompt

        try:
            async with self._session.post(
                f"{self.config.base_url}/api/generate",
                json=payload,
            ) as resp:
                async for line in resp.content:
                    if line:
                        data = json.loads(line)
                        if "response" in data:
                            yield data["response"]
                        if data.get("done"):
                            break
        except Exception as e:
            logger.error(f"Stream error: {e}")
            yield f"[ERROR: {e}]"

    # =========================================================================
    # Robot-specific convenience methods
    # =========================================================================

    async def analyze_scene(
        self,
        description: str,
        image: Optional[bytes] = None,
    ) -> AIResponse:
        """
        Analyze a scene for navigation and obstacle detection.

        Args:
            description: Text description of sensor data
            image: Optional camera image for visual analysis

        Returns:
            AIResponse with scene analysis
        """
        system_prompt = """You are a robot vision system. Analyze the scene and provide:
1. Identified objects and obstacles with positions
2. Navigable paths and areas
3. Potential hazards or concerns
4. Recommended actions

Be concise and structured. Format as JSON when possible."""

        return await self.generate(
            prompt=description,
            task_type=TaskType.SCENE_ANALYSIS,
            system_prompt=system_prompt,
            images=[image] if image else None,
        )

    async def plan_path(
        self,
        current_position: dict,
        goal_position: dict,
        obstacles: list[dict],
        constraints: Optional[dict] = None,
    ) -> AIResponse:
        """
        Get AI assistance for complex path planning scenarios.

        Args:
            current_position: Robot's current x, y, theta
            goal_position: Target x, y, theta
            obstacles: List of obstacle positions and shapes
            constraints: Optional movement constraints

        Returns:
            AIResponse with path planning suggestions
        """
        prompt = f"""Path planning request:
Current: {json.dumps(current_position)}
Goal: {json.dumps(goal_position)}
Obstacles: {json.dumps(obstacles)}
Constraints: {json.dumps(constraints or {})}

Suggest waypoints and approach strategy."""

        system_prompt = """You are a robot path planning assistant. Analyze the scenario
and provide waypoint suggestions, considering:
- Obstacle avoidance
- Smooth trajectory
- Robot kinematic constraints
- Energy efficiency

Return structured waypoints as JSON array."""

        return await self.generate(
            prompt=prompt,
            task_type=TaskType.PATH_PLANNING,
            system_prompt=system_prompt,
        )

    async def process_command(self, natural_language: str) -> AIResponse:
        """
        Process natural language commands into robot actions.

        Args:
            natural_language: User's verbal command

        Returns:
            AIResponse with parsed intent and actions
        """
        system_prompt = """You are a robot command interpreter. Parse the user's
natural language into structured robot commands. Output JSON with:
{
  "intent": "string - what the user wants",
  "action": "string - primary action type",
  "parameters": {},
  "confidence": 0.0-1.0
}

Action types: navigate, pickup, follow, stop, return_home, explore, scan, report"""

        return await self.generate(
            prompt=natural_language,
            task_type=TaskType.NATURAL_LANGUAGE,
            system_prompt=system_prompt,
        )

    async def identify_object(
        self,
        description: str,
        image: Optional[bytes] = None,
    ) -> AIResponse:
        """
        Identify and classify an object.

        Args:
            description: Sensor-based description of the object
            image: Optional camera image

        Returns:
            AIResponse with object identification
        """
        system_prompt = """You are a robot object recognition system. Identify the object and provide:
{
  "name": "object name",
  "category": "category",
  "confidence": 0.0-1.0,
  "properties": {"color": "", "size": "", "material": ""},
  "is_obstacle": true/false,
  "is_graspable": true/false,
  "suggested_action": "string"
}"""

        return await self.generate(
            prompt=description,
            task_type=TaskType.OBJECT_IDENTIFICATION,
            system_prompt=system_prompt,
            images=[image] if image else None,
        )

    async def make_decision(
        self,
        situation: str,
        options: list[str],
        context: Optional[dict] = None,
    ) -> AIResponse:
        """
        Get AI assistance for complex decision making.

        Args:
            situation: Description of the current situation
            options: Available action options
            context: Additional context (battery, time, mission, etc.)

        Returns:
            AIResponse with recommended decision
        """
        prompt = f"""Decision required:
Situation: {situation}
Options: {json.dumps(options)}
Context: {json.dumps(context or {})}

Analyze and recommend the best option with reasoning."""

        system_prompt = """You are a robot decision support system. Analyze the situation
and recommend the best action. Consider:
- Safety (highest priority)
- Mission objectives
- Resource constraints (battery, time)
- Risk assessment

Output JSON:
{{
  "recommended_option": "string",
  "confidence": 0.0-1.0,
  "reasoning": "brief explanation",
  "risks": ["list of risks"],
  "fallback": "alternative if primary fails"
}}"""

        return await self.generate(
            prompt=prompt,
            task_type=TaskType.DECISION_MAKING,
            system_prompt=system_prompt,
        )

    @property
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected
