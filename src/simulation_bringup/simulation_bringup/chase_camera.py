"""Enable Gazebo's local-frame chase camera once the GUI and robot are ready."""
import json
import os
import subprocess
import time

from .model import load_config


def main():
    cfg = load_config(os.environ["PIROBOT_SIM_CONFIG"])
    distance = cfg["gazebo"].get("chase_distance", 1.0)
    height = cfg["gazebo"].get("chase_height", 0.4)
    expected_height = cfg["gazebo"]["spawn"][2] + height
    request = ('track_mode: FOLLOW follow_target: {name: "burger"} '
               f'follow_offset: {{x: {-distance} y: 0 z: {height}}} '
               'follow_pgain: 0.15 track_pgain: 0.15')
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        try:
            subprocess.run(['gz', 'topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack',
                            '-p', request], check=True, timeout=3, capture_output=True)
            status = subprocess.run(['gz', 'topic', '-t', '/gui/currently_tracked', '-e',
                                     '-n', '1', '--json-output'],
                                    check=True, timeout=3, capture_output=True, text=True)
            data = json.loads(status.stdout)
            if (data.get('trackMode') == 'FOLLOW' and
                    data.get('followTarget', {}).get('name') == 'burger' and
                    abs(data.get('followOffset', {}).get('x', 0) + distance) < .01):
                pose = subprocess.run(['gz', 'topic', '-t', '/gui/camera/pose', '-e',
                                       '-n', '1', '--json-output'], check=True, timeout=3,
                                      capture_output=True, text=True)
                # A tracking request can be acknowledged before the renderer has
                # spawned the target, then cleared. Confirm actual camera motion.
                if abs(json.loads(pose.stdout).get('position', {}).get('z', 6) - expected_height) > 0.15:
                    time.sleep(.5)
                    continue
                print(f'Gazebo: camera dietro al robot attiva ({distance} m dietro, {height} m sopra).', flush=True)
                return
        except (subprocess.SubprocessError, ValueError):
            pass
        time.sleep(.5)
    print('Gazebo: camera automatica non disponibile; usare Follow dal menu del robot.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
