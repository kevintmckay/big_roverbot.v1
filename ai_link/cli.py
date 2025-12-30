#!/usr/bin/env python3
"""
AI-Link CLI: Command-line interface for testing AI-Link.

Usage:
    python -m ai_link.cli --host 192.168.1.100 --prompt "Analyze this scene"
    python -m ai_link.cli --config config.json --prompt "What should I do?"
"""

import asyncio
import argparse
import json
import sys

from .client import AILinkClient, TaskType
from .config import AILinkConfig


async def main():
    parser = argparse.ArgumentParser(
        description="AI-Link CLI - Test connection to Ollama server"
    )

    # Connection options
    parser.add_argument("--host", default="192.168.1.100", help="Ollama server host")
    parser.add_argument("--port", type=int, default=11434, help="Ollama server port")
    parser.add_argument("--config", help="Path to config JSON file")

    # TLS options
    parser.add_argument("--no-tls", action="store_true", help="Disable TLS")
    parser.add_argument("--no-verify", action="store_true", help="Skip SSL verification")
    parser.add_argument("--ca-cert", help="Path to CA certificate")
    parser.add_argument("--client-cert", help="Path to client certificate")
    parser.add_argument("--client-key", help="Path to client key")

    # Model options
    parser.add_argument("--model", default="qwen2.5:32b", help="Model to use")

    # Commands
    subparsers = parser.add_subparsers(dest="command", help="Command to run")

    # Status command
    subparsers.add_parser("status", help="Check connection status")

    # Models command
    subparsers.add_parser("models", help="List available models")

    # Generate command
    gen_parser = subparsers.add_parser("generate", help="Generate text")
    gen_parser.add_argument("prompt", help="Prompt to send")
    gen_parser.add_argument("--system", help="System prompt")
    gen_parser.add_argument("--stream", action="store_true", help="Stream output")

    # Scene analysis command
    scene_parser = subparsers.add_parser("scene", help="Analyze scene")
    scene_parser.add_argument("description", help="Scene description")
    scene_parser.add_argument("--image", help="Path to image file")

    # Command processing
    cmd_parser = subparsers.add_parser("command", help="Process natural language")
    cmd_parser.add_argument("text", help="Natural language command")

    # Decision making
    decision_parser = subparsers.add_parser("decide", help="Get decision support")
    decision_parser.add_argument("situation", help="Situation description")
    decision_parser.add_argument("--options", nargs="+", required=True, help="Options")

    # Generate config
    cfg_parser = subparsers.add_parser("init-config", help="Generate config file")
    cfg_parser.add_argument("output", help="Output file path")

    args = parser.parse_args()

    # Build config
    if args.config:
        config = AILinkConfig.from_file(args.config)
    else:
        config = AILinkConfig(
            host=args.host,
            port=args.port,
            use_tls=not args.no_tls,
            verify_ssl=not args.no_verify,
            ca_cert=args.ca_cert,
            client_cert=args.client_cert,
            client_key=args.client_key,
            default_model=args.model,
        )

    # Handle init-config separately
    if args.command == "init-config":
        config.to_file(args.output)
        print(f"Config saved to {args.output}")
        return

    # Create client and run command
    async with AILinkClient(config) as client:
        if not client.is_connected:
            print("ERROR: Failed to connect to Ollama server", file=sys.stderr)
            sys.exit(1)

        if args.command == "status":
            print(f"Connected: {client.is_connected}")
            print(f"Server: {config.base_url}")
            print(f"TLS: {config.use_tls}")
            print(f"Model: {config.default_model}")

        elif args.command == "models":
            models = await client.list_models()
            print("Available models:")
            for m in models:
                print(f"  - {m}")

        elif args.command == "generate":
            if args.stream:
                async for token in client.generate_stream(
                    args.prompt, system_prompt=args.system
                ):
                    print(token, end="", flush=True)
                print()
            else:
                response = await client.generate(
                    args.prompt, system_prompt=args.system
                )
                print(response.text)
                print(f"\n[{response.model}, {response.tokens_used} tokens, {response.duration_ms:.0f}ms]")

        elif args.command == "scene":
            image_data = None
            if args.image:
                with open(args.image, "rb") as f:
                    image_data = f.read()

            response = await client.analyze_scene(args.description, image_data)
            print(response.text)

        elif args.command == "command":
            response = await client.process_command(args.text)
            print(response.text)

        elif args.command == "decide":
            response = await client.make_decision(
                args.situation, args.options
            )
            print(response.text)

        else:
            parser.print_help()


def run():
    asyncio.run(main())


if __name__ == "__main__":
    run()
