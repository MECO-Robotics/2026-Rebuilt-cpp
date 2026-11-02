#!/usr/bin/env python3
"""Publish a USB gamepad's drive axes to a robot over NT4.

Driver Station remains responsible for robot enable state and all buttons.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="JSON settings file (overridden by explicit CLI flags)")
    parser.add_argument("--team", type=int, help="FRC team number; connects to roboRIO at 10.TE.AM.2")
    parser.add_argument("--server", help="NT4 server host/IP; overrides --team")
    parser.add_argument("--port", type=int, help="NT4 server port (default: 5810)")
    parser.add_argument("--rate", type=float, help="publish rate in Hz (default: 200)")
    parser.add_argument("--joystick", type=int, help="pygame joystick index (default: 0)")
    parser.add_argument("--left-y", type=int, help="axis index for forward/backward (default: 1)")
    parser.add_argument("--left-x", type=int, help="axis index for left/right (default: 0)")
    parser.add_argument("--right-x", type=int, help="axis index for rotation (default: 2)")
    parser.add_argument("--raw-axes", action=argparse.BooleanOptionalAction, default=None,
                        help="use raw joystick axes instead of SDL's standardized gamepad mapping")
    parser.add_argument("--invert-left-y", action=argparse.BooleanOptionalAction, default=None, help="invert the left Y axis")
    parser.add_argument("--invert-left-x", action=argparse.BooleanOptionalAction, default=None, help="invert the left X axis")
    parser.add_argument("--invert-right-x", action=argparse.BooleanOptionalAction, default=None, help="invert the right X axis")
    parser.add_argument("--deadband", type=float, help="radial-independent axis deadband from 0 to <1 (default: 0)")
    parser.add_argument("--topic", help="NT4 topic (default: /Input/Gamepad)")
    parser.add_argument("--print-only", action="store_true", help="read and display controller without connecting to NT")
    args = parser.parse_args()
    cli_team = "--team" in sys.argv
    cli_server = "--server" in sys.argv
    cli_axis_map = any(flag in sys.argv for flag in ("--left-y", "--left-x", "--right-x"))

    cfg: dict = {}
    config_path = args.config or (Path(os.environ["FRC_NT_GAMEPAD_CONFIG"]) if os.getenv("FRC_NT_GAMEPAD_CONFIG") else None)
    if config_path:
        try:
            cfg = json.loads(config_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            parser.error(f"cannot read config {config_path}: {exc}")
        if not isinstance(cfg, dict):
            parser.error("config must contain a JSON object")
        allowed = {"team", "server", "port", "rate", "joystick", "raw_axes", "left_y", "left_x", "right_x",
                   "deadband", "topic", "invert_left_y", "invert_left_x", "invert_right_x"}
        extra = sorted(set(cfg) - allowed)
        if extra:
            parser.error(f"unknown config key(s): {', '.join(extra)}")
    defaults = {"team": 8324, "port": 5810, "rate": 200.0, "joystick": 0,
                "left_y": 1, "left_x": 0, "right_x": 2, "deadband": 0.0,
                "topic": "/Input/Gamepad", "invert_left_y": False,
                "invert_left_x": False, "invert_right_x": False}
    explicit_config_axes = any(name in cfg for name in ("left_y", "left_x", "right_x"))
    config_raw_axes = cfg.get("raw_axes", explicit_config_axes)
    if args.raw_axes is None:
        args.raw_axes = config_raw_axes if config_path else cli_axis_map
    if cli_axis_map:
        args.raw_axes = True
    for key, default in defaults.items():
        if getattr(args, key, None) is None:
            setattr(args, key, cfg.get(key, default))
    if not args.server and not cli_team and not cli_server:
        args.server = cfg.get("server")
    if args.team is not None and (type(args.team) is not int or not 1 <= args.team <= 99999):
        parser.error("--team must be in [1, 99999]")
    if args.server is not None and (not isinstance(args.server, str) or not args.server.strip()):
        parser.error("server must be a nonempty hostname or IP")
    if args.server is None and args.team is None:
        parser.error("set --team or --server")
    if type(args.port) is not int or not 1 <= args.port <= 65535:
        parser.error("port must be in [1, 65535]")
    if isinstance(args.rate, bool) or not isinstance(args.rate, (int, float)) or not (0 < args.rate <= 200) or not math.isfinite(args.rate):
        parser.error("rate must be in (0, 200]")
    if isinstance(args.deadband, bool) or not isinstance(args.deadband, (int, float)) or not (0 <= args.deadband < 1) or not math.isfinite(args.deadband):
        parser.error("deadband must be in [0, 1)")
    if type(args.joystick) is not int or args.joystick < 0:
        parser.error("joystick index must be a nonnegative integer")
    for axis in (args.left_x, args.left_y, args.right_x):
        if type(axis) is not int or axis < 0:
            parser.error("axis indices must be nonnegative integers")
    for name in ("invert_left_y", "invert_left_x", "invert_right_x"):
        if type(getattr(args, name)) is not bool:
            parser.error(f"{name} must be true or false")
    if type(args.raw_axes) is not bool:
        parser.error("raw_axes must be true or false")
    if not isinstance(args.topic, str) or not args.topic.startswith("/"):
        parser.error("topic must be an absolute NetworkTables topic name")
    return args


def main() -> int:
    args = parse_args()
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    try:
        import pygame
    except ImportError:
        raise SystemExit("Install a Pygame SDL2 build in this Python environment: python -m pip install 'pygame-ce>=2.5.8'")
    if not args.print_only:
        try:
            import ntcore
        except ImportError:
            raise SystemExit("Install RobotPy NTCore: python -m pip install pyntcore")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.joystick:
        raise SystemExit(f"No gamepad at joystick index {args.joystick}; connect it by USB and retry")
    pad = pygame.joystick.Joystick(args.joystick)
    pad.init()
    gamepad = None
    if not args.raw_axes:
        try:
            from pygame._sdl2 import controller as sdl_controller
        except ImportError as exc:
            pygame.quit()
            raise SystemExit("This Pygame build lacks SDL GameController support; install pygame-ce >=2.5.8 or pass --raw-axes with explicit mappings") from exc
        sdl_controller.init()
        if not sdl_controller.is_controller(args.joystick):
            pygame.quit()
            raise SystemExit(
                f"{pad.get_name()} has no SDL GameController mapping. Add a mapping or pass --raw-axes "
                "with --left-y/--left-x/--right-x after inspecting it with --print-only --raw-axes."
            )
        gamepad = sdl_controller.Controller(args.joystick)
        # SDL's standardized axis constants are re-exported at pygame's top
        # level (pygame-ce and pygame 2.6), not from _sdl2.controller.
        axis_map = (pygame.CONTROLLER_AXIS_LEFTY,
                    pygame.CONTROLLER_AXIS_LEFTX,
                    pygame.CONTROLLER_AXIS_RIGHTX)
        print(f"Using SDL GameController mapping: {pad.get_name()}")
    else:
        indices = (args.left_y, args.left_x, args.right_x)
        if max(indices) >= pad.get_numaxes():
            pygame.quit()
            raise SystemExit(f"{pad.get_name()} has {pad.get_numaxes()} axes; configured index {max(indices)} is unavailable")
        axis_map = indices
        print(f"Using raw joystick axes: {pad.get_name()}")
    instance_id = pad.get_instance_id()

    inst = None
    publisher = None
    if not args.print_only:
        inst = ntcore.NetworkTableInstance.create()
        inst.startClient4("usb-gamepad")
        host = args.server or f"10.{args.team // 100}.{args.team % 100}.2"
        inst.setServer(host, args.port)
        print(f"Connecting to NT4 {host}:{args.port}; controller: {pad.get_name()} ({pad.get_numaxes()} axes)")
        deadline = time.monotonic() + 15
        while not inst.isConnected() and time.monotonic() < deadline:
            time.sleep(.05)
        if not inst.isConnected():
            inst.stopClient()
            ntcore.NetworkTableInstance.destroy(inst)
            raise SystemExit(f"Could not connect to {host}:{args.port}; verify network and robot NT server")
        while inst.getServerTimeOffset() is None:
            if time.monotonic() > deadline:
                raise SystemExit("NT4 clock synchronization did not complete")
            time.sleep(.01)
        publisher = inst.getDoubleArrayTopic(args.topic).publish(
            ntcore.PubSubOptions(periodic=1.0 / args.rate, sendAll=True))
        offset = inst.getServerTimeOffset()
        if offset is None:
            raise SystemExit("NT4 server time synchronization was lost")
        # Seed above a previous run's sequence so robot-side monotonic checking
        # accepts the first sample after a normal publisher restart.
        sequence = max(0, ntcore._now() + offset)
    else:
        sequence = 0

    def filtered(axis: int, invert: bool) -> float:
        raw = gamepad.get_axis(axis) if gamepad is not None else pad.get_axis(axis)
        if gamepad is not None:
            value = raw / 32767.0 if raw >= 0 else raw / 32768.0
        else:
            value = float(raw)
        if not math.isfinite(value):
            return 0.0
        if invert:
            value = -value
        value = max(-1.0, min(1.0, value))
        if abs(value) <= args.deadband:
            return 0.0
        return math.copysign((abs(value) - args.deadband) / (1.0 - args.deadband), value)

    print("Publishing axes only. Enable/disable and buttons remain on Driver Station. Ctrl-C to stop.")
    period = 1.0 / args.rate
    due = time.monotonic()
    try:
        while True:
            pygame.event.pump()
            # pygame can keep returning cached axis values after USB removal.
            # Stop publishing at the removal event so the robot freshness gate
            # drops the sample and commands zero.
            removed = False
            for event in pygame.event.get():
                if event.type in (pygame.JOYDEVICEREMOVED,
                                  getattr(sdl_controller, "CONTROLLERDEVICEREMOVED", -1) if gamepad is not None else -1) and \
                        getattr(event, "instance_id", None) == instance_id:
                    removed = True
            controller_detached = gamepad is not None and (not gamepad.get_init() or not gamepad.attached())
            joystick_detached = not pad.get_init() or (hasattr(pad, "get_attached") and not pad.get_attached())
            if removed or controller_detached or joystick_detached:
                print("\nConfigured gamepad disconnected; stopped publishing. Reconnect and restart the script.")
                break
            axes = (filtered(axis_map[0], args.invert_left_y),
                    filtered(axis_map[1], args.invert_left_x),
                    filtered(axis_map[2], args.invert_right_x))
            if publisher is not None:
                local_us = ntcore._now()
                offset = inst.getServerTimeOffset()
                if offset is None:
                    raise RuntimeError("NT4 server time synchronization was lost")
                source_us = local_us + offset
                sequence += 1
                # NTCore converts client-local publisher timestamps to server
                # time. Embed that same synchronized server timestamp.
                publisher.set([1.0, float(sequence), float(source_us), *axes], local_us)
                inst.flush()
            else:
                print(f"leftY={axes[0]:+.3f} leftX={axes[1]:+.3f} rightX={axes[2]:+.3f}     ", end="\r", flush=True)
            due += period
            now = time.monotonic()
            if due < now:
                due = now + period
            time.sleep(max(0.0, due - time.monotonic()))
    except KeyboardInterrupt:
        print("\nStopping gamepad publisher")
    finally:
        if publisher is not None:
            publisher.close()
        if inst is not None:
            inst.stopClient()
            ntcore.NetworkTableInstance.destroy(inst)
        if gamepad is not None:
            gamepad.quit()
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
