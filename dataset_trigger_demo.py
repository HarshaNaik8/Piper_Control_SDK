"""Automated Dataset Collection Integration Script.

Demonstrates how to integrate the Piper controller into your dataset collection
pipeline so you can trigger robot trajectory replays automatically across multiple
episodes without any manual button presses.

Usage:
  # Replay a recorded software trajectory across 5 episodes:
  python dataset_trigger_demo.py --trajectory trajectories/my_trajectory.json --episodes 5

  # Replay in simulation (offline test):
  python dataset_trigger_demo.py --episodes 3 --mock
"""

import os
import sys
import glob
import time
import argparse
from typing import Optional

from piper_controller import PiperRobotController


def run_dataset_collection(
    num_episodes: int = 5,
    trajectory_file: Optional[str] = None,
    interface: str = "agx_cando",
    channel: str = "0",
    speed_factor: float = 1.0,
    mock: bool = False,
    episode_delay: float = 2.0,
):
    print("=" * 70)
    print(" AUTOMATED PIPER DATASET COLLECTION RUNNER")
    print(f" Target Episodes : {num_episodes}")
    print(f" Interface       : {interface}:{channel} {'(SIMULATION)' if mock else ''}")
    print("=" * 70)

    # 1. Resolve trajectory file
    if not trajectory_file:
        saved = glob.glob("trajectories/*.json")
        if saved:
            saved.sort(key=os.path.getmtime, reverse=True)
            trajectory_file = saved[0]
            print(f"[Dataset] Using latest recorded trajectory: {trajectory_file}")

    # 2. Initialize and connect controller
    robot = PiperRobotController(
        interface=interface,
        channel=channel,
        mock=mock,
    )

    if not robot.connect(timeout=3.0):
        print("[ERROR] Failed to connect to Piper robot. Check connection.")
        return 1

    # 3. Power on and wake up arm
    print("[+] Waking up robot arm and enabling all motors (0xFF)...")
    robot.enable_arm()
    time.sleep(1.0)

    try:
        for episode in range(1, num_episodes + 1):
            print("\n" + "-" * 50)
            print(f">>> STARTING EPISODE {episode} / {num_episodes}")
            print("-" * 50)

            print(f"[Camera/Sensors] >>> RECORDING STARTED for Episode {episode} <<<")

            if trajectory_file and os.path.exists(trajectory_file):
                print(f"[Dataset] Replaying trajectory: {os.path.basename(trajectory_file)}")
                robot.replay_software_trajectory(
                    trajectory=trajectory_file,
                    speed_factor=speed_factor,
                    loop_count=1,
                    blocking=True,
                )
            else:
                print("[Dataset] Triggering firmware trajectory replay (CAN 0x150, 0x03)...")
                robot.execute_firmware_trajectory()
                time.sleep(3.0)

            print(f"[Camera/Sensors] >>> RECORDING STOPPED for Episode {episode} <<<")
            print(f"[Dataset] Episode {episode} data captured and saved.")

            if episode < num_episodes:
                print(f"[Dataset] Waiting {episode_delay}s before next episode...")
                time.sleep(episode_delay)

        print("\n" + "=" * 70)
        print(f"[SUCCESS] Dataset collection run complete ({num_episodes} episodes executed).")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\n[INFO] Dataset collection interrupted by user.")
        robot.stop_software_replay()
    finally:
        print("[+] Disconnecting robot cleanly...")
        robot.disconnect()

    return 0


def main():
    parser = argparse.ArgumentParser(description="Piper Dataset Collection Loop")
    parser.add_argument("--episodes", type=int, default=3, help="Number of episodes to record")
    parser.add_argument("--trajectory", type=str, default=None, help="Path to JSON trajectory file")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed factor (0.5 to 2.0)")
    parser.add_argument("--interface", type=str, default="agx_cando", help="CAN interface")
    parser.add_argument("--channel", type=str, default="0", help="CAN channel")
    parser.add_argument("--delay", type=float, default=2.0, help="Delay between episodes in seconds")
    parser.add_argument("--mock", action="store_true", help="Simulate execution without physical robot")
    args = parser.parse_args()

    sys.exit(
        run_dataset_collection(
            num_episodes=args.episodes,
            trajectory_file=args.trajectory,
            interface=args.interface,
            channel=args.channel,
            speed_factor=args.speed,
            mock=args.mock,
            episode_delay=args.delay,
        )
    )


if __name__ == "__main__":
    main()
