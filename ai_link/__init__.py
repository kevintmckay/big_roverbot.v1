"""
AI-Link: Secure communication module for offloading complex AI tasks
to a remote Ollama server over encrypted connection.
"""

from .client import AILinkClient
from .config import AILinkConfig

__version__ = "0.1.0"
__all__ = ["AILinkClient", "AILinkConfig"]
