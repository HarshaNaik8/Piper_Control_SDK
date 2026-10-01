"""Automated Dataset Collection Integration Script.

Demonstrates how to integrate the Piper Sync Trigger into the multi-camera
video recording pipeline so you can trigger robot trajectory replays automatically
across multiple episodes without any manual button presses or latency error.

Usage:
  # Run 5 automated replay episodes with 3s delay:
  python dataset_trigger_demo.py --episodes 5 --delay 3.0

  # Replay in simulation (offline test):
  python dataset_trigger_demo.py --episodes 3 --mock
"""

import sys
import time
import argparse
from typing import Optional

from piper_sync_trigger import PiperSyncTrigger


def run_dataset_collection(
    num_episodes: int = 5,
    interface: str = "agx_cando",
    channel: str = "0",
    mock: bool = False,
    episode_delay: float = 2.0,
):
    print("=" * 70)
    print(" AUTOMATED PIPER DATASET COLLECTION RUNNER (CAMERA SYNC)")
    print(f" Target Episodes : {num_episodes}")
    print(f" Interface       : {interface}:{channel} {'(SIMULATION)' if mock else ''}")
    print("=" * 70)

    # 1. Initialize and connect controller
    robot = PiperSyncTrigger(
        interface=interface,
        channel=channel,
        mock=mock,
    )

    if not robot.connect(timeout=3.0):
        print("[ERROR] Failed to connect to Piper robot. Check connection.")
        return 1

    try:
        for episode in range(1, num_episodes + 1):
            print("\n" + "-" * 50)
            print(f">>> STARTING EPISODE {episode} / {num_episodes}")
            print("-" * 50)

            # --- SENSOR / CAMERA START ---
            print(f"[Mentor Script] >>> 3x SAMSUNG PHONES: RECORDING STARTED for Episode {episode} <<<")
            
            # --- ROBOT TRIGGER (Zero ms delay!) ---
            print(f"[Piper] >>> Triggering Trajectory Playback (CAN 0x150, grag_teach_ctrl=0x03)...")
            start_t = time.time()
            finished = robot.replay_and_wait(timeout=60.0)
            duration = time.time() - start_t

            # --- SENSOR / CAMERA STOP ---
            print(f"[Mentor Script] >>> 3x SAMSUNG PHONES: RECORDING STOPPED for Episode {episode} <<<")
            print(f"[Dataset] Episode {episode} captured in {duration:.2f}s (Replay Status: {'OK' if finished else 'TIMEOUT'}).")
            print(f"[Mentor Script] Shifting ISO/exposure parameters for next episode...")

            if episode < num_episodes:
                print(f"[Dataset] Waiting {episode_delay}s before next episode...")
                time.sleep(episode_delay)

        print("\n" + "=" * 70)
        print(f"[SUCCESS] Dataset collection run complete ({num_episodes} episodes executed).")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\n[INFO] Dataset collection interrupted by user.")
        pass
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
