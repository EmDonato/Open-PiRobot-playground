#!/usr/bin/env bash
set -eo pipefail
# ROS Jazzy's binary extensions require the system Python (Ubuntu 24.04).
export PATH="/usr/bin:$PATH"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${PIROBOT_SIM_CONFIG:-$ROOT/config/simulation.yaml}"
config_value() {
  python3 - "$CONFIG" "$1" <<'PYCFG'
import sys, yaml
value=yaml.safe_load(open(sys.argv[1]))
for part in sys.argv[2].split("."):
    value=value[part]
print(str(value).lower() if isinstance(value,bool) else value)
PYCFG
}
usage() {
  cat <<'EOF'
Uso: ./scripts/sim.sh COMANDO [--teleop joystick|keyboard|none] [--docker] [--gpu nvidia|none] [argomenti ROS]
Comandi: up, build, check, assets, test, smoke, test-teleop, test-controls, check-camera, logs
  --teleop keyboard  guida dal terminale corrente (WASD, spazio stop, Ctrl-C per uscire)
  --teleop joystick  joystick L1 + stick (default)
  --docker          esegue in Docker; build costruisce anche l'immagine
  --gpu nvidia      Docker con NVIDIA Container Toolkit
  --gpu none        Docker senza NVIDIA, rendering software (default Docker)
Esempi:
  ./scripts/sim.sh up --teleop keyboard
  ./scripts/sim.sh build --docker
  ./scripts/sim.sh up --docker --gpu nvidia --teleop joystick
  ./scripts/sim.sh up --docker --gpu none --teleop keyboard
  ./scripts/sim.sh up slam:=false  # in RViz selezionare Fixed Frame=odom
EOF
}
COMMAND="${1:-up}"
(($# == 0)) || shift
TELEOP="$(config_value teleop.default)"
DOCKER="$(config_value docker.use)"
GPU="$(config_value docker.gpu)"
GPU_EXPLICIT=false
ARGS=()
while (($#)); do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --docker) DOCKER=true; shift ;;
    --no-docker) DOCKER=false; shift ;;
    --teleop|--gpu)
      option="$1"
      [[ $# -ge 2 ]] || { echo "Valore mancante: $option" >&2; exit 2; }
      if [[ "$option" == --teleop ]]; then TELEOP="$2"; else GPU="$2"; GPU_EXPLICIT=true; fi
      shift 2 ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
case "$COMMAND" in help|-h|--help) usage; exit 0 ;; esac
case "$TELEOP" in joystick|keyboard|none) ;; *) echo 'Teleop: joystick|keyboard|none' >&2; exit 2 ;; esac
case "$GPU" in nvidia|none) ;; *) echo 'GPU: nvidia|none' >&2; exit 2 ;; esac
if [[ "$GPU_EXPLICIT" == true && "$DOCKER" != true ]]; then
  echo '--gpu richiede --docker' >&2; exit 2
fi
if [[ "$DOCKER" == true ]]; then
  IMAGE="${PIROBOT_SIM_IMAGE:-$(config_value docker.image)}"
  if [[ "$COMMAND" == build ]]; then
    docker build -f "$ROOT/docker/simulation.Dockerfile" -t "$IMAGE" "$ROOT"
  fi
  options=(--rm --init --network host --ipc host --user "$(id -u):$(id -g)"
    --env HOME=/tmp --env DISPLAY --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(config_value runtime.ros_domain_id)}"
    --env QT_X11_NO_MITSHM=1 --env PIROBOT_SIM_WS="$ROOT/$(config_value docker.workspace)"
    --volume "$ROOT:$ROOT" --workdir "$ROOT"
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
  [[ -z "${PIROBOT_SIM_CONFIG:-}" ]] || options+=(--env PIROBOT_SIM_CONFIG)
  [[ -z "${GZ_PARTITION:-}" ]] || options+=(--env GZ_PARTITION)
  if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
    options+=(--volume "$XAUTHORITY:/tmp/sim.xauth:ro" --env XAUTHORITY=/tmp/sim.xauth)
  fi
  [[ ! -t 0 || ! -t 1 ]] || options+=(-it)
  if [[ "$TELEOP" == joystick && -d /dev/input ]]; then
    options+=(--volume /dev/input:/dev/input --device-cgroup-rule 'c 13:* rwm')
    for group in $(id -G); do options+=(--group-add "$group"); done
  fi
  if [[ "$GPU" == nvidia ]]; then
    options+=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute --env LIBGL_ALWAYS_SOFTWARE=0)
  else
    options+=(--env LIBGL_ALWAYS_SOFTWARE=1)
  fi
  exec docker run "${options[@]}" "$IMAGE" bash "$ROOT/scripts/sim.sh" "$COMMAND" --teleop "$TELEOP" "${ARGS[@]}"
fi
set -- "$COMMAND" "${ARGS[@]}"
WS="${PIROBOT_SIM_WS:-$ROOT/.simulation/local}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(config_value runtime.ros_domain_id)}"
export GZ_PARTITION="${GZ_PARTITION:-pirobot_sim_${ROS_DOMAIN_ID}}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/ros_logs}"
export PIROBOT_SIM_CONFIG="$CONFIG"
export PYTHONPATH="$ROOT/src/simulation_bringup${PYTHONPATH:+:$PYTHONPATH}"
source /opt/ros/jazzy/setup.bash
case "${1:-up}" in
  assets)
    python3 -m simulation_bringup.assets "$CONFIG"
    ;;
  build)
    bash "$ROOT/scripts/sim.sh" assets
    bash "$ROOT/scripts/sim.sh" check
    mkdir -p "$WS"
    mapfile -t SOURCES < <(python3 - "$CONFIG" <<'PY'
import sys
from simulation_bringup.model import load_config
for component in load_config(sys.argv[1])['components'].values():
    if component.get('enabled', True) and component.get('source'):
        print(component['source'])
PY
    )
    colcon --log-base "$WS/log" build --base-paths \
      "$ROOT/src/robot_descriptor" "$ROOT/src/simulation_bringup" \
      "$ROOT/interfaces/src" "${SOURCES[@]}" \
      --build-base "$WS/build" --install-base "$WS/install" \
      --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
    ;;
  up)
    bash "$ROOT/scripts/sim.sh" assets
    if [[ ! -f "$WS/install/setup.bash" ]]; then
      echo 'Run ./scripts/sim.sh build first (add --docker when using Docker).' >&2; exit 1
    fi
    source "$WS/install/setup.bash"
    shift || true
    slam_enabled=true
    for arg in "$@"; do [[ "$arg" != slam:=false ]] || slam_enabled=false; done
    if [[ "$slam_enabled" == true ]] && ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
      echo 'Manca SLAM Toolbox: installare ros-jazzy-slam-toolbox oppure usare --docker dopo ./scripts/sim.sh build --docker.' >&2
      exit 1
    fi
    if [[ "$TELEOP" != joystick ]]; then
      for arg in "$@"; do
        [[ "$arg" != joystick:=true ]] || { echo 'joystick:=true confligge con --teleop' >&2; exit 2; }
      done
      set -- "$@" joystick:=false
    fi
    if [[ "$TELEOP" == keyboard ]]; then
      [[ -t 0 ]] || { echo 'La tastiera richiede un terminale interattivo.' >&2; exit 2; }
      mkdir -p "$ROS_LOG_DIR"
      # Bash background jobs inherit SIGINT ignored; reset it before ROS launch.
      python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp("ros2", ["ros2", *sys.argv[1:]])' launch simulation_bringup simulation.launch.py config:="$CONFIG" "$@" >"$ROS_LOG_DIR/keyboard-launch.log" 2>&1 &
      launch_pid=$!
      trap 'kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true' EXIT
      echo "Log simulazione: $ROS_LOG_DIR/keyboard-launch.log"
      python3 -m simulation_bringup.keyboard --ros-args -p config_file:="$CONFIG"
    else
      exec ros2 launch simulation_bringup simulation.launch.py config:="$CONFIG" "$@"
    fi
    ;;
  check)
    python3 - "$CONFIG" <<'PY'
import sys
from pathlib import Path
from simulation_bringup.model import load_config
cfg = load_config(sys.argv[1])
if not Path(cfg['slam']['parameter_file']).is_file():
    raise SystemExit('Missing SLAM parameter file')
for name, c in cfg['components'].items():
    if c.get('enabled', True):
        for p in c.get('parameter_files', []) + ([c['source']] if 'source' in c else []):
            if not Path(p).exists():
                raise SystemExit(f'{name}: missing path {p}')
print('Simulation configuration valid')
PY
    ;;
  test)
    source "$WS/install/setup.bash"
    python3 -m unittest discover -s "$ROOT/tests" -p 'test_simulation.py' -v
    ;;
  check-camera)
    source "$WS/install/setup.bash"
    exec python3 "$ROOT/tests/camera_alignment_check.py"
    ;;
  test-controls)
    source "$WS/install/setup.bash"
    python3 "$ROOT/tests/control_motion_smoke.py" keyboard
    python3 "$ROOT/tests/control_motion_smoke.py" joystick
    ;;
  test-teleop)
    source "$WS/install/setup.bash"
    exec python3 "$ROOT/tests/teleop_smoke.py"
    ;;
  smoke)
    bash "$ROOT/scripts/sim.sh" assets
    source "$WS/install/setup.bash"
    exec python3 "$ROOT/tests/simulation_smoke.py"
    ;;
  logs)
    tail -n 100 "$WS/smoke.log"
    ;;
  *) echo 'Usage: bash scripts/sim.sh {assets|check|build|test|test-teleop|check-camera|smoke|up [gui:=false]}'; exit 2 ;;
esac
