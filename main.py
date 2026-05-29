from __future__ import annotations

import argparse
import time
from typing import Callable


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control a Unitree G1 from Python using unitree_sdk2_python.",
    )
    parser.add_argument(
        "command",
        choices=[
            "damp",
            "squat-up",
            "squat-down",
            "sit",
            "low-stand",
            "high-stand",
            "zero-torque",
            "stop",
            "move",
            "lie-up",
            "walk-square",
            "walk-circle",
            "bow",
            "sprint",
            "zigzag",
            "spin",
            "pump",
            "moonwalk",
            "tornado",
            "shuffle",
            "figure-eight",
            "stomp",
            "crab-walk",
            "ballet",
            "dance",
            "party",
            "demo",
            "say",
            "led",
            "volume",
            "greet",
            "announce",
            "rave",
            "snapshot",
            "stream",
        ],
        help="G1 action to execute.",
    )
    parser.add_argument(
        "--interface",
        default=None,
        help="Network interface connected to the robot, for example enp2s0.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="RPC timeout in seconds.",
    )
    parser.add_argument(
        "--vx",
        type=float,
        default=0.0,
        help="Forward velocity for the move command in m/s.",
    )
    parser.add_argument(
        "--vy",
        type=float,
        default=0.0,
        help="Lateral velocity for the move command in m/s.",
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="Yaw rate for the move command in rad/s.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="Move duration in seconds when not using --continuous.",
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Keep the move command active until another command is sent.",
    )
    parser.add_argument(
        "--text",
        type=str,
        default="Hello, I am the G1 robot!",
        help="Text for the say/announce command.",
    )
    parser.add_argument(
        "--color",
        type=str,
        default="0,0,255",
        help="RGB color for the led command, e.g. '255,0,0' for red.",
    )
    parser.add_argument(
        "--volume",
        type=int,
        default=80,
        help="Volume level (0-100) for the volume command.",
    )
    parser.add_argument(
        "--speaker-id",
        type=int,
        default=1,
        help="TTS speaker/language ID (0=Chinese, 1=English, 2=German via Piper).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would run without touching the robot.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Acknowledge that the robot is clear of obstacles and safe to command.",
    )
    return parser


def load_sdk():
    try:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as exc:
        raise SystemExit(
            "unitree_sdk2py is not installed. Install the official Unitree SDK first. "
            "See README.md for setup steps, including CycloneDDS."
        ) from exc

    return ChannelFactoryInitialize, ChannelSubscriber, LocoClient, LowState_


def load_audio_sdk():
    try:
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        return AudioClient
    except ImportError:
        return None


def require_confirmation(args: argparse.Namespace) -> None:
    if args.dry_run:
        return
    if args.yes:
        return
    raise SystemExit(
        "Refusing to send robot commands without --yes. Use --dry-run to inspect the command first."
    )


def initialize_client(args: argparse.Namespace):
    channel_factory_initialize, channel_subscriber, loco_client_type, low_state_type = (
        load_sdk()
    )

    if args.interface:
        channel_factory_initialize(0, args.interface)
    else:
        channel_factory_initialize(0)

    # Audio-only commands don't need the LocoClient / LowState subscription
    audio_only = args.command in ("say", "led", "volume")
    # Camera-only commands don't need any robot client
    camera_only = args.command in ("snapshot", "stream")

    audio_client = None
    audio_client_type = load_audio_sdk()
    if audio_client_type is not None:
        audio_client = audio_client_type()
        audio_client.SetTimeout(args.timeout)
        audio_client.Init()

    if camera_only:
        return None, audio_client

    if audio_only:
        if audio_client is None:
            raise SystemExit("AudioClient not available in this SDK build.")
        return None, audio_client

    client = loco_client_type()
    client.SetTimeout(args.timeout)
    client.Init()

    # Startup commands (used when robot is freshly powered on) skip the
    # LowState wait because the body controller may not publish lowstate
    # until the robot is in an active FSM state.
    startup_commands = {"squat-up", "damp", "zero-torque", "lie-up"}
    if args.command in startup_commands:
        print(f"Startup command '{args.command}' — skipping LowState wait.")
        return client, audio_client

    # Subscribe to LowState and wait for robot status before sending commands
    state_received = [False]

    def on_low_state(msg):
        state_received[0] = True

    subscriber = channel_subscriber("rt/lowstate", low_state_type)
    subscriber.Init(on_low_state, 10)

    print("Waiting for robot state...")
    deadline = time.time() + args.timeout
    while not state_received[0]:
        if time.time() > deadline:
            raise SystemExit(
                "Timeout waiting for robot state. Is the robot powered on?"
            )
        time.sleep(0.1)
    print("Robot state received.")

    return client, audio_client


def describe_command(args: argparse.Namespace) -> str:
    if args.command == "move":
        if args.continuous:
            duration_text = "continuous"
        else:
            duration_text = f"{args.duration:.2f}s"
        return (
            f"command=move interface={args.interface or '<default>'} "
            f"vx={args.vx:.3f} vy={args.vy:.3f} yaw={args.yaw:.3f} duration={duration_text}"
        )

    return f"command={args.command} interface={args.interface or '<default>'}"


UINT32_MAX = (1 << 32) - 1


def _enter_walk_mode(client):
    """Transition through FSM states into walk mode."""
    print("Setting prepare mode (FSM 4)...")
    client.SetFsmId(4)
    time.sleep(2.0)
    print("Setting walk mode (FSM 500)...")
    client.SetFsmId(500)
    time.sleep(2.0)


def _pump(client, reps=4, speed=0.3):
    """Rapid body height pumping."""
    for i in range(reps):
        client.SetStandHeight(0)
        time.sleep(speed)
        client.SetStandHeight(UINT32_MAX)
        time.sleep(speed)


def _speak(audio, text, speaker_id):
    """Speak text using TtsMaker (Chinese/English) or Piper TTS (German)."""
    if speaker_id == 2:
        _speak_piper(audio, text)
    else:
        code = audio.TtsMaker(text, speaker_id)
        print(f"TTS result: {code}")


def _speak_piper(audio, text):
    """Generate German speech with Piper TTS and stream via PlayStream."""
    import subprocess, tempfile, os, struct as st

    piper_bin = "/home/unitree/tts_service/piper/piper"
    model_path = "/home/unitree/tts_service/de_DE-thorsten-medium.onnx"

    if not os.path.exists(piper_bin):
        print("Piper TTS not installed on robot. Falling back to TtsMaker.")
        audio.TtsMaker(text, 0)
        return

    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    try:
        result = subprocess.run(
            [piper_bin, "--model", model_path, "--output_file", tmp.name],
            input=text.encode("utf-8"),
            capture_output=True,
        )
        if result.returncode != 0:
            print(f"Piper error: {result.stderr.decode()}")
            return

        # Minimal WAV reader (16-bit PCM)
        with open(tmp.name, "rb") as f:
            raw = f.read()
        # Find 'data' chunk
        idx = raw.find(b"data")
        if idx == -1:
            print("Invalid WAV output")
            return
        data_size = st.unpack_from("<I", raw, idx + 4)[0]
        pcm = raw[idx + 8 : idx + 8 + data_size]

        # Read sample rate from WAV header (offset 24)
        sample_rate = st.unpack_from("<I", raw, 24)[0]

        # Resample to 16kHz if needed
        if sample_rate != 16000:
            samples = st.unpack(f"<{len(pcm)//2}h", pcm)
            ratio = sample_rate / 16000
            resampled = [samples[int(i * ratio)] for i in range(int(len(samples) / ratio))]
            pcm = st.pack(f"<{len(resampled)}h", *resampled)

        # Stream in chunks
        stream_id = str(int(time.time() * 1000))
        chunk_size = 32000  # 1 second at 16kHz mono 16-bit
        offset = 0
        total_chunks = 0
        while offset < len(pcm):
            chunk = pcm[offset : offset + chunk_size]
            ret, _ = audio.PlayStream("g1_tts", stream_id, chunk)
            if ret != 0:
                print(f"PlayStream error: {ret}")
                break
            offset += chunk_size
            total_chunks += 1
            time.sleep(0.3)
        # Wait for playback to finish before returning
        remaining_secs = (len(pcm) / 32000) + 1.0
        time.sleep(remaining_secs)
    finally:
        os.unlink(tmp.name)


def execute_command(client, audio, args: argparse.Namespace) -> None:
    if args.command == "say":
        print(f"Speaking: {args.text}")
        _speak(audio, args.text, args.speaker_id)
        return
    elif args.command == "led":
        parts = [int(x) for x in args.color.split(",")]
        r, g, b = parts[0], parts[1], parts[2]
        print(f"Setting LED to R={r} G={g} B={b}")
        code = audio.LedControl(r, g, b)
        print(f"LED result: {code}")
        return
    elif args.command == "volume":
        print(f"Setting volume to {args.volume}%")
        code = audio.SetVolume(args.volume)
        print(f"Volume result: {code}")
        code2, data = audio.GetVolume()
        print(f"Current volume: {data}")
        return

    if args.command == "damp":
        client.Damp()
    elif args.command == "squat-up":
        client.Damp()
        time.sleep(0.5)
        client.Squat2StandUp()
    elif args.command == "squat-down":
        client.StandUp2Squat()
    elif args.command == "sit":
        client.Sit()
    elif args.command == "low-stand":
        client.LowStand()
    elif args.command == "high-stand":
        client.HighStand()
    elif args.command == "zero-torque":
        client.ZeroTorque()
    elif args.command == "stop":
        client.StopMove()
    elif args.command == "move":
        _enter_walk_mode(client)
        print(f"Moving: vx={args.vx}, vy={args.vy}, yaw={args.yaw}")
        client.SetVelocity(args.vx, args.vy, args.yaw, args.duration)
        if not args.continuous:
            time.sleep(args.duration + 0.5)
            client.StopMove()
    elif args.command == "walk-square":
        _enter_walk_mode(client)
        side = args.duration  # reuse --duration as side length in seconds
        speed = 0.3
        for i in range(4):
            print(f"Square side {i+1}/4: forward...")
            client.SetVelocity(speed, 0, 0, side)
            time.sleep(side + 0.5)
            print(f"Square side {i+1}/4: turning 90 degrees...")
            client.SetVelocity(0, 0, 0.78, 2.0)  # ~90 deg in 2s
            time.sleep(2.5)
        client.StopMove()
    elif args.command == "walk-circle":
        _enter_walk_mode(client)
        print("Walking in a circle...")
        client.SetVelocity(0.2, 0, 0.5, args.duration)  # forward + constant yaw
        time.sleep(args.duration + 0.5)
        client.StopMove()
    elif args.command == "bow":
        print("Bowing...")
        client.HighStand()
        time.sleep(2.0)
        client.LowStand()
        time.sleep(1.5)
        client.HighStand()
        time.sleep(1.5)
    elif args.command == "sprint":
        _enter_walk_mode(client)
        print("Sprinting forward!")
        client.SetVelocity(0.8, 0, 0, args.duration)
        time.sleep(args.duration + 0.5)
        client.StopMove()
    elif args.command == "zigzag":
        _enter_walk_mode(client)
        print("Zigzag run!")
        for i in range(6):
            vy = 0.5 if i % 2 == 0 else -0.5
            print(f"  Zig {i+1}/6")
            client.SetVelocity(0.5, vy, 0, 1.2)
            time.sleep(1.5)
        client.StopMove()
    elif args.command == "spin":
        _enter_walk_mode(client)
        print("Spinning!")
        client.SetVelocity(0, 0, 2.5, args.duration)
        time.sleep(args.duration + 0.5)
        client.StopMove()
    elif args.command == "pump":
        print("Pumping body up and down!")
        _pump(client, reps=6, speed=0.4)
    elif args.command == "moonwalk":
        _enter_walk_mode(client)
        print("Moonwalking!")
        # Walk backward while slowly rotating
        client.SetVelocity(-0.3, 0, 0.4, args.duration)
        time.sleep(args.duration + 0.5)
        client.StopMove()
    elif args.command == "tornado":
        _enter_walk_mode(client)
        print("Tornado! Spinning while circling!")
        # Walk in a circle while spinning fast
        client.SetVelocity(0.3, 0.3, 2.0, args.duration)
        time.sleep(args.duration + 0.5)
        client.StopMove()
    elif args.command == "shuffle":
        _enter_walk_mode(client)
        print("Shuffling!")
        for i in range(6):
            vy = 0.5 if i % 2 == 0 else -0.5
            print(f"  Shuffle {i+1}/6")
            client.SetVelocity(0, vy, 0, 0.8)
            time.sleep(1.0)
        client.StopMove()
    elif args.command == "figure-eight":
        _enter_walk_mode(client)
        print("Figure-eight!")
        # First loop: curve right
        print("  Loop 1: curving right")
        client.SetVelocity(0.3, 0, -0.5, 6.0)
        time.sleep(6.5)
        # Second loop: curve left
        print("  Loop 2: curving left")
        client.SetVelocity(0.3, 0, 0.5, 6.0)
        time.sleep(6.5)
        client.StopMove()
    elif args.command == "stomp":
        print("Stomping!")
        # Rapid sit-stand transitions that look like stomping
        for i in range(4):
            print(f"  Stomp {i+1}/4")
            client.SetStandHeight(0)
            time.sleep(0.25)
            client.SetStandHeight(UINT32_MAX)
            time.sleep(0.25)
            client.SetStandHeight(0)
            time.sleep(0.25)
            client.SetStandHeight(UINT32_MAX)
            time.sleep(0.5)
    elif args.command == "crab-walk":
        _enter_walk_mode(client)
        print("Crab walking sideways!")
        client.SetVelocity(0, 0.5, 0, args.duration)
        time.sleep(args.duration + 0.5)
        client.StopMove()
    elif args.command == "ballet":
        _enter_walk_mode(client)
        print("=== Ballet routine ===")
        print("  Rise tall")
        client.StopMove()
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.5)
        print("  Slow pirouette")
        client.SetVelocity(0, 0, 0.8, 4.0)
        time.sleep(4.5)
        print("  Glide forward")
        client.SetVelocity(0.15, 0, 0, 2.0)
        time.sleep(2.5)
        print("  Plié (dip low)")
        client.StopMove()
        client.SetStandHeight(0)
        time.sleep(1.5)
        print("  Rise and turn")
        client.SetStandHeight(UINT32_MAX)
        time.sleep(0.5)
        client.SetVelocity(0, 0, -0.8, 4.0)
        time.sleep(4.5)
        print("  Glide back")
        client.SetVelocity(-0.15, 0, 0, 2.0)
        time.sleep(2.5)
        print("  Final plié")
        client.StopMove()
        client.SetStandHeight(0)
        time.sleep(1.0)
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.0)
        print("=== Ballet complete ===")
    elif args.command == "dance":
        _enter_walk_mode(client)
        print("=== Dance routine ===")
        for i in range(3):
            print(f"  Beat {i*4+1}: Sway left + bob down")
            client.SetVelocity(0, 0.3, 0.3, 1.0)
            client.SetStandHeight(0)
            time.sleep(1.0)
            print(f"  Beat {i*4+2}: Sway right + bob up")
            client.SetVelocity(0, -0.3, -0.3, 1.0)
            client.SetStandHeight(UINT32_MAX)
            time.sleep(1.0)
            print(f"  Beat {i*4+3}: Step forward low")
            client.SetVelocity(0.4, 0, 0, 0.8)
            client.SetStandHeight(0)
            time.sleep(0.8)
            print(f"  Beat {i*4+4}: Step back high")
            client.SetVelocity(-0.3, 0, 0, 0.8)
            client.SetStandHeight(UINT32_MAX)
            time.sleep(0.8)
        print("  Breakdown: fast shuffle!")
        for j in range(4):
            client.SetVelocity(0, 0.5 if j % 2 == 0 else -0.5, 0, 0.5)
            time.sleep(0.6)
        print("  Finale: spin + pump!")
        client.SetVelocity(0, 0, 2.5, 4.0)
        for _ in range(5):
            client.SetStandHeight(0)
            time.sleep(0.4)
            client.SetStandHeight(UINT32_MAX)
            time.sleep(0.4)
        client.StopMove()
        print("=== Dance complete ===")
    elif args.command == "party":
        print("=== PARTY MODE ===")
        _enter_walk_mode(client)
        # 1. Sprint in
        print("  Sprint entrance!")
        client.SetVelocity(0.8, 0, 0, 2.0)
        time.sleep(2.5)
        # 2. Skid stop + spin
        print("  Skid stop and spin!")
        client.SetVelocity(0, 0, 2.5, 3.0)
        time.sleep(3.5)
        # 3. Pump while shuffling
        print("  Pump and shuffle!")
        for i in range(4):
            client.SetVelocity(0, 0.5 if i % 2 == 0 else -0.5, 0, 0.8)
            client.SetStandHeight(0 if i % 2 == 0 else UINT32_MAX)
            time.sleep(0.8)
        # 4. Zigzag sprint
        print("  Zigzag sprint!")
        for i in range(4):
            client.SetVelocity(0.6, 0.4 if i % 2 == 0 else -0.4, 0, 1.0)
            time.sleep(1.2)
        # 5. Tornado
        print("  Tornado spin!")
        client.SetVelocity(0.3, 0.3, 2.5, 4.0)
        time.sleep(4.5)
        # 6. Moonwalk back
        print("  Moonwalk back!")
        client.SetVelocity(-0.4, 0, 0.5, 3.0)
        time.sleep(3.5)
        # 7. Final: rapid pumps + spin
        print("  Grand finale!")
        client.SetVelocity(0, 0, 2.0, 5.0)
        for _ in range(8):
            client.SetStandHeight(0)
            time.sleep(0.3)
            client.SetStandHeight(UINT32_MAX)
            time.sleep(0.3)
        client.StopMove()
        # 8. Bow
        print("  Take a bow!")
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.0)
        client.SetStandHeight(0)
        time.sleep(1.0)
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.0)
        # 9. Sit
        client.SetFsmId(3)
        time.sleep(1.0)
        print("=== PARTY OVER ===")
    elif args.command == "demo":
        print("=== Full Demo ===")
        # Stand up tall
        print("Step 1: Rise up")
        client.SetStandHeight(UINT32_MAX)
        time.sleep(2.0)
        # Pump
        print("Step 2: Body pump")
        _pump(client, reps=3, speed=0.4)
        # Walk mode
        _enter_walk_mode(client)
        print("Step 3: Walk forward")
        client.SetVelocity(0.3, 0, 0, 2.0)
        time.sleep(2.5)
        print("Step 4: Sidestep right")
        client.SetVelocity(0, -0.4, 0, 1.5)
        time.sleep(2.0)
        print("Step 5: Walk backward")
        client.SetVelocity(-0.3, 0, 0, 2.0)
        time.sleep(2.5)
        print("Step 6: Sidestep left")
        client.SetVelocity(0, 0.4, 0, 1.5)
        time.sleep(2.0)
        print("Step 7: Spin!")
        client.SetVelocity(0, 0, 2.0, 3.0)
        time.sleep(3.5)
        print("Step 8: Pump while spinning!")
        client.SetVelocity(0, 0, 1.5, 3.0)
        for _ in range(4):
            client.SetStandHeight(0)
            time.sleep(0.35)
            client.SetStandHeight(UINT32_MAX)
            time.sleep(0.35)
        client.StopMove()
        print("Step 9: Bow")
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.0)
        client.SetStandHeight(0)
        time.sleep(1.0)
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.0)
        print("Step 10: Sit")
        client.SetFsmId(3)
        time.sleep(2.0)
        print("=== Demo complete ===")
    elif args.command == "lie-up":
        client.Damp()
        time.sleep(0.5)
        client.Lie2StandUp()
    elif args.command == "greet":
        # Stand tall, light up, speak, and bow
        print("=== Greeting ===")
        if audio:
            audio.LedControl(0, 255, 0)  # Green
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.5)
        if audio:
            _speak(audio, args.text or "Hello! Nice to meet you! I am the G1 robot.", args.speaker_id)
        time.sleep(3.0)
        # Bow
        client.SetStandHeight(0)
        time.sleep(1.5)
        client.SetStandHeight(UINT32_MAX)
        time.sleep(1.0)
        if audio:
            audio.LedControl(0, 0, 255)  # Back to blue
        print("=== Greeting complete ===")
    elif args.command == "announce":
        # LED flash + speak custom text
        print(f"=== Announcing: {args.text} ===")
        if audio:
            # Flash red-white-blue
            for r, g, b in [(255, 0, 0), (255, 255, 255), (0, 0, 255)]:
                audio.LedControl(r, g, b)
                time.sleep(0.5)
            _speak(audio, args.text, args.speaker_id)
            time.sleep(3.0)
            audio.LedControl(0, 0, 255)
        print("=== Announcement complete ===")
    elif args.command == "rave":
        # LED light show + movement dance
        print("=== RAVE MODE ===")
        _enter_walk_mode(client)
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255),
            (255, 255, 0), (255, 0, 255), (0, 255, 255),
            (255, 128, 0), (128, 0, 255),
        ]
        if audio:
            audio.SetVolume(100)
            _speak(audio, "Let's rave!", args.speaker_id)
            time.sleep(2.0)
        # Fast color cycling + movement
        for i in range(16):
            r, g, b = colors[i % len(colors)]
            if audio:
                audio.LedControl(r, g, b)
            # Alternate movements
            if i % 4 == 0:
                client.SetVelocity(0, 0, 2.5, 1.0)
                client.SetStandHeight(0)
            elif i % 4 == 1:
                client.SetVelocity(0, 0.5, 0, 1.0)
                client.SetStandHeight(UINT32_MAX)
            elif i % 4 == 2:
                client.SetVelocity(0, 0, -2.5, 1.0)
                client.SetStandHeight(0)
            else:
                client.SetVelocity(0, -0.5, 0, 1.0)
                client.SetStandHeight(UINT32_MAX)
            time.sleep(0.5)
        client.StopMove()
        if audio:
            audio.LedControl(0, 0, 255)
            _speak(audio, "That was fun!", args.speaker_id)
        print("=== RAVE OVER ===")
    elif args.command == "snapshot":
        _camera_snapshot()
    elif args.command == "stream":
        _camera_stream(args)


def _camera_snapshot():
    """Capture an RGB + depth frame from the RealSense D435I and save to /tmp."""
    import pyrealsense2 as rs
    import numpy as np
    import cv2

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        pipe.start(cfg)
    except RuntimeError as e:
        raise SystemExit(f"Camera error: {e}. Is the RealSense plugged in?")

    # Let auto-exposure settle
    for _ in range(30):
        pipe.wait_for_frames()

    frames = pipe.wait_for_frames()
    color = np.asarray(frames.get_color_frame().get_data())
    depth = np.asarray(frames.get_depth_frame().get_data())

    cv2.imwrite("/tmp/g1_rgb.jpg", color)

    # Colour-map depth for visualization
    depth_clip = np.clip(depth, 0, 6000)
    depth8 = cv2.convertScaleAbs(depth_clip, alpha=255.0 / 6000)
    depth_color = cv2.applyColorMap(depth8, cv2.COLORMAP_PLASMA)
    cv2.imwrite("/tmp/g1_depth.jpg", depth_color)

    pipe.stop()
    print(f"RGB: /tmp/g1_rgb.jpg ({color.shape})")
    print(f"Depth: /tmp/g1_depth.jpg (range {depth.min()}-{depth.max()}mm)")


def _camera_stream(args):
    """Stream RealSense as MJPEG over HTTP — viewable in any browser."""
    import pyrealsense2 as rs
    import numpy as np
    import cv2
    import threading
    from http.server import HTTPServer, BaseHTTPRequestHandler

    w, h, fps = 640, 480, 15
    port = 8080

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)

    try:
        pipe.start(cfg)
    except RuntimeError as e:
        raise SystemExit(f"Camera error: {e}. Is the RealSense plugged in?")

    # Let auto-exposure settle
    for _ in range(15):
        pipe.wait_for_frames()

    temp_filter = rs.temporal_filter()
    latest_frame = [None]
    lock = threading.Lock()
    running = [True]

    def capture_loop():
        count = 0
        while running[0]:
            frames = pipe.wait_for_frames()
            color = np.asarray(frames.get_color_frame().get_data())

            depth = frames.get_depth_frame()
            depth = temp_filter.process(depth)
            depth16 = np.asarray(depth.get_data())
            depth_clip = np.clip(depth16, 0, 6000)
            depth8 = cv2.convertScaleAbs(depth_clip, alpha=255.0 / 6000)
            depth_color = cv2.applyColorMap(depth8, cv2.COLORMAP_PLASMA)

            # Side-by-side: RGB | Depth
            combined = cv2.hconcat([color, depth_color])
            _, jpeg = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_frame[0] = jpeg.tobytes()
            count += 1
            if count % 300 == 0:
                print(f"  Captured {count} frames...")

    class MJPEGHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(
                    b'<html><body style="margin:0;background:#111">'
                    b'<img src="/stream" style="width:100%;height:auto">'
                    b'</body></html>'
                )
            elif self.path == "/stream":
                self.send_response(200)
                boundary = "frame"
                self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
                self.end_headers()
                try:
                    while running[0]:
                        with lock:
                            frame = latest_frame[0]
                        if frame is None:
                            time.sleep(0.05)
                            continue
                        self.wfile.write(f"--{boundary}\r\n".encode())
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        time.sleep(1.0 / fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif self.path == "/snapshot":
                with lock:
                    frame = latest_frame[0]
                if frame:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                else:
                    self.send_response(503)
                    self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, format, *a):
            pass  # suppress per-request logs

    cap_thread = threading.Thread(target=capture_loop, daemon=True)
    cap_thread.start()

    server = HTTPServer(("0.0.0.0", port), MJPEGHandler)
    print(f"Live stream: http://192.168.123.164:{port}/")
    print(f"Snapshot:    http://192.168.123.164:{port}/snapshot")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping stream...")
    finally:
        running[0] = False
        server.server_close()
        pipe.stop()


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    require_confirmation(args)
    print(describe_command(args))

    if args.dry_run:
        return

    client, audio = initialize_client(args)
    execute_command(client, audio, args)
    print("Command sent successfully.")


if __name__ == "__main__":
    main()
