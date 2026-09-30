"""Automated Dataset Collection Integration Script.

Demonstrates how to integrate the Piper controller into your dataset collection
pipeline so you can trigger robot trajectory replays automatically across multiple
episodes without any manual button presses.

Usage:
  # With physical robot:
  python dataset_trigger_demo.py --episodes 5

  # In simulation (offline test):
  python dataset_trigger_demo.py --episodes 3 --mock
"""

import sys
import time
import argparse
from typing import Optional

from piper_controller import PiperRobotController


def run_dataset_collection(
    num_episodes: int = 5,
    interface: str = "agx_cando",
    channel: str = "0",
    mock: bool = False,
    episode_delay: float = 2.0,
):
    print("=" * 70)
    print(" AUTOMATED PIPER DATASET COLLECTION RUNNER")
    print(f" Target Episodes : {num_episodes}")
    print(f" Interface       : {interface}:{channel} {'(SIMULATION)' if mock else ''}")
    print("=" * 70)

    # 1. Initialize and connect controller
    robot = PiperRobotController(
        interface=interface,
        channel=channel,
        mock=mock,
    )

    if not robot.connect(timeout=3.0):
        print("[ERROR] Failed to connect to Piper robot. Check connection.")
        return 1

    # 2. Safety enable
    print("[+] Enabling robot arm motors...")
    robot.enable_arm()
    time.sleep(1.0)

    try:
        for episode in range(1, num_episodes + 1):
            print("\n" + "-" * 50)
            print(f">>> STARTING EPISODE {episode} / {num_episodes}")
            print("-" * 50)

            # Optional: Hook to start your camera/sensor recording
            print(f"[Dataset] Triggering sensor & camera recording for Episode {episode}...")

            # Trigger the recorded trajectory (software double-tap equivalent)
            print("[Dataset] Triggering Piper trajectory execution...")
            if not robot.execute_taught_trajectory():
                print(f"[!] Failed to trigger trajectory on episode {episode}.")
                break

            # Wait for the trajectory to complete automatically
            # You can also sample joint positions in the callback during playback!
            def telemetry_callback(status):
                joints = robot.get_joint_angles()
                # print(f"  [Recording] Status: {status['arm_status_name']} | Joints: {joints}")

            completed = robot.wait_for_trajectory_completion(
                timeout=90.0,
                poll_interval=0.1,
                on_progress=telemetry_callback,
            )

            if not completed:
                print(f"[WARN] Episode {episode} did not complete within the timeout or was interrupted.")
                break

            # Optional: Hook to save / end episode dataset file
            print(f"[Dataset] Episode {episode} finished! Saved episode data.")

            if episode < num_episodes:
                print(f"[Dataset] Waiting {episode_delay}s before next episode...")
                time.sleep(episode_delay)

        print("\n" + "=" * 70)
        print(f"[SUCCESS] Dataset collection run complete ({num_episodes} episodes executed).")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\n[INFO] Dataset collection interrupted by user.")
        robot.stop_trajectory()
    finally:
        print("[+] Disconnecting robot cleanly...")
        robot.disconnect()

    return 0


def main():
    parser = argparse.ArgumentParser(description="Piper Dataset Collection Loop")
    parser.add_argument("--episodes", type=int, default=3, help="Number of episodes to record")
    parser.add_argument("--interface", type=str, default="agx_cando", help="CAN interface")
    parser.add_argument("--channel", type=str, default="0", help="CAN channel")
    parser.add_argument("--delay", type=float, default=2.0, help="Delay between episodes in seconds")
    parser.add_argument("--mock", action="store_true", help="Simulate execution without physical robot")
    args = parser.parse_args()

    sys.exit(
        run_dataset_collection(
            num_episodes=args.episodes,
            interface=args.interface,
            channel=args.channel,
            mock=args.mock,
            episode_delay=args.delay,
        )
    )


if __name__ == "__main__":
    main()
