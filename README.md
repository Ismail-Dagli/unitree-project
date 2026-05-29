# Unitree G1 Python Starter

This repository gives you a minimal Python entry point for controlling a Unitree G1 through the official `unitree_sdk2_python` SDK.

## What is in this repo

- `main.py`: a small command-line controller for common G1 locomotion actions
- `pyproject.toml`: project metadata and a `unitree-g1` console script

## Important constraints

- The official Python SDK is not a pure-Python package. It depends on CycloneDDS and usually needs to be installed from the official source repository.
- You should run robot control from a Linux environment with direct access to the robot network interface. WSL may work for development, but direct Ethernet access from native Linux is the safer baseline.
- Do not test motion commands unless the robot has clearance around it.

## 1. Install system prerequisites

On Ubuntu:

```bash
sudo apt update
sudo apt install -y git cmake build-essential python3-pip
```

## 2. Install CycloneDDS

The official Unitree SDK documents CycloneDDS `0.10.x` as the dependency line.

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

## 3. Install the Unitree Python SDK

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
export CYCLONEDDS_HOME=~/cyclonedds/install
pip3 install -e .
```

If the SDK install fails with a CycloneDDS lookup error, verify that `CYCLONEDDS_HOME` points at the `install` directory you built above.

For this repository's `uv run` workflow, keep `CYCLONEDDS_HOME` exported in your shell and use Python 3.12. The `pyproject.toml` in this repo is configured to use the sibling checkout at `~/unitree_sdk2_python` as an editable dependency.

## 4. Find the robot network interface

List interfaces:

```bash
ip link show
```

Typical names are `enp2s0`, `enx...`, or `eth0`. Use the interface physically connected to the G1.

## 5. Run the controller in this repo

Examples:

```bash
export CYCLONEDDS_HOME=~/cyclonedds/install
python main.py squat-up --interface enp2s0 --dry-run
python main.py squat-up --interface enp2s0 --yes
python main.py move --interface enp2s0 --vx 0.15 --duration 2.0 --yes
python main.py stop --interface enp2s0 --yes
python main.py wave-hand --interface enp2s0 --yes
```

If you install this project as a package, you can also use:

```bash
unitree-g1 damp --interface enp2s0 --yes
```

## Supported commands

- `damp`
- `squat-up`
- `squat-down`
- `sit`
- `low-stand`
- `high-stand`
- `zero-torque`
- `stop`
- `move`
- `wave-hand`
- `wave-hand-turn`
- `shake-hand`
- `lie-up`

## Notes on G1 API usage

- G1 locomotion uses `unitree_sdk2py.g1.loco.g1_loco_client.LocoClient`
- The older script in this repo used a `g1.sport.sport_client` import, which does not match the current official SDK structure
- `move` sends velocity commands with `vx`, `vy`, and `yaw`; this repo stops the robot automatically after `--duration` unless you pass `--continuous`

## Recommended next steps

Once basic control works, the next useful additions are:

1. Add a state reader to subscribe to robot status.
2. Add scripted motion sequences instead of one-off commands.
3. Split the code into a small reusable `G1Controller` class if you want to build applications on top of it.
