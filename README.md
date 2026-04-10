# 🤖 Multirobot Adaptive Control with Reinforcement Learning

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b%2B-blue.svg)](https://mathworks.com)
[![Language](https://img.shields.io/badge/Language-MATLAB-orange.svg)]()

> **Coordinated control of a KUKA KR10 industrial robot and two RoArm-M2-S painting arms using deep reinforcement learning for adaptive pick-and-place, spray painting, and real-time collision avoidance.**

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Pipeline](#pipeline)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [RL Agents](#rl-agents)
- [PLC Integration](#plc-integration)
- [License](#license)

---

## Overview

This project implements a **multi-phase reinforcement learning pipeline** that trains three heterogeneous robots to work together in a shared workspace:

| Robot | Task | RL Algorithm |
|-------|------|-------------|
| **KUKA KR10 R1100** | Pick-and-place workpiece handling | TD3 |
| **RoArm-M2-S #1** | Spray painting (speed profile) | TD3 |
| **RoArm-M2-S #2** | Spray painting (speed profile) | TD3 |
| **All 3 together** | Collision avoidance | DDPG |

The key insight is that **collision avoidance must be trained on data from robots already running their learned policies** — not random motion. The pipeline enforces this dependency chain automatically.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     MATLAB Pipeline                         │
│                                                             │
│  Phase 1: KUKA RL ──► Phase 2: RoArm RL                    │
│              │                    │                         │
│              └────────┬───────────┘                         │
│                       ▼                                     │
│          Phase 3: Live Multi-Robot Collection               │
│           (all 3 robots run trained policies)               │
│                       │                                     │
│                       ▼                                     │
│          Phase 4: Collision Avoidance RL                    │
├─────────────────────────────────────────────────────────────┤
│  Communication Layer                                        │
│  ┌──────────┐  ┌───────────────┐  ┌──────────────────────┐ │
│  │ KUKA TCP │  │ RoArm HTTP/   │  │ PLC OPC-UA           │ │
│  │ (KVP)    │  │ WiFi JSON API │  │ (Siemens S7-1500)    │ │
│  └──────────┘  └───────────────┘  └──────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware

| Component | Model | Connection | Default Address |
|-----------|-------|-----------|-----------------|
| Industrial Robot | KUKA KR10 R1100 | TCP/KVP | `172.31.17.101:7000` |
| Painting Arm 1 | RoArm-M2-S | WiFi HTTP | `192.168.1.192` |
| Painting Arm 2 | RoArm-M2-S | WiFi HTTP | `192.168.1.101` |
| PLC | Siemens S7-1500 | OPC-UA | `opc.tcp://172.31.17.1:4840` |
| Simulink Model | KukaSimModel | Local | — |

---

## Pipeline

The full pipeline is executed via a single command:

```matlab
results = run_full_multirobot_pipeline();          % synthetic mode (default)
results = run_full_multirobot_pipeline(cfg);        % custom configuration
```

### Phase Dependency Chain

```
Phase 0: Setup ─► Phase 1: KUKA RL ─► Phase 2: RoArm RL
                                              │
                  ┌───────────────────────────┘
                  ▼
          Phase 3: Live Collection (GATE: Phase 1 & 2 must PASS)
                  │
                  ▼
          Phase 4: Collision RL   (GATE: Phase 3 must PASS)
                  │
                  ▼
          Phase 5: Summary
```

> ⚠️ **Phase 3 is blocked** if either Phase 1 or Phase 2 fails. This is intentional — collision data must reflect trained robot behaviours, not random motion.

### Modes

| Mode | Description |
|------|-------------|
| `synthetic` | Simulated robot positions with Gaussian noise. No hardware required. |
| `online` | Connects to real KUKA (TCP) and RoArm (WiFi) hardware for live data collection. |

---

## Repository Structure

```
.
├── README.md
├── LICENSE                              # MIT License
├── Software_Architecture.ini            # System architecture configuration
├── Lead_Compensetor_Solution.m          # Lead compensator design
│
├── MATLAB/
│   ├── run.m                            # ★ Main pipeline entry point
│   ├── setup.m                          # Path and environment setup
│   ├── PLC_OPC_UA_Launcher.m            # OPC-UA PLC interface
│   ├── writeRobotDataToPLC.m            # Write robot state to PLC
│   ├── practise8.m                      # Practice/demo script
│   ├── KukaSimModel.slxc               # Simulink KUKA model (compiled)
│   │
│   ├── KUKA_control/                    # KUKA KR10 control modules
│   │   ├── create_pick_and_place_trajectory.m
│   │   ├── Pick_car_trajectory.m
│   │   ├── KukaTest_Connection.m
│   │   ├── Environment_setup.m
│   │   ├── convert_trajectory_points.m
│   │   ├── Kuka6DoF.mlx                # Live script: 6-DOF kinematics
│   │   ├── KukaKR.mlx                  # Live script: KR model
│   │   ├── KukaKR60.mlx                # Live script: KR60 model
│   │   ├── MatLabControl.src           # KRL source for KUKA controller
│   │   ├── MatLabControl.dat           # KRL data file
│   │   └── srcs/                       # Additional KUKA source files
│   │
│   ├── Roarm_control/                   # RoArm-M2-S control modules
│   │   ├── RoArm_Painting_System.m     # Full painting system
│   │   ├── RoarmM2_MotionControl.m     # Serial motion control
│   │   ├── RoarmM2_MotionControl_WiFi.m# WiFi motion control
│   │   ├── RoarmM2_Demo.m             # Quick demo
│   │   ├── RoarmM2_CompleteExample.m   # Complete usage example
│   │   ├── GETTING_STARTED.m          # Setup guide
│   │   ├── QuickReference.m           # API reference
│   │   └── INDEX.m                     # Module index
│   │
│   ├── RL/                              # Reinforcement Learning
│   │   ├── Main.m                       # RL entry point
│   │   ├── runKukaPickPlaceRL.m         # KUKA TD3 training script
│   │   ├── runRoarmPaintingRL.m         # RoArm TD3 training script
│   │   ├── KukaPickPlaceEnv.m           # KUKA RL environment
│   │   ├── RoarmPaintingEnv.m           # RoArm RL environment
│   │   ├── CollisionAvoidanceEnv.m      # Collision RL environment
│   │   ├── collect_collision_dataset.m  # Multi-robot data collector
│   │   ├── train_collision_agent.m      # DDPG collision training
│   │   ├── evaluate_collision_agent.m   # Collision agent evaluation
│   │   ├── evaluate_roarm_agent.m       # RoArm agent evaluation
│   │   ├── collision_reward.m           # Collision reward function
│   │   ├── build_collision_state.m      # State builder for collision env
│   │   ├── build_rl_state.m             # Generic RL state builder
│   │   ├── analyse_reward.m             # Reward analysis utilities
│   │   ├── hyperparam_sweep.m           # Hyperparameter search
│   │   ├── kuka_position_rl_dataset.csv # KUKA training data
│   │   ├── roarm_position_rl_dataset.csv# RoArm training data
│   │   ├── savedAgents/                 # Trained agent checkpoints (.mat)
│   │   └── figures/                     # Training plots and figures
│   │
│   ├── Mathmatical Models/              # Mathematical models & derivations
│   └── slprj/                           # Simulink project cache
│
├── Hardware/                            # Hardware documentation & schematics
├── PLC/                                 # PLC programs (Siemens TIA Portal)
└── docs/                                # Additional documentation
```

---

## Prerequisites

- **MATLAB R2023b** or later
- **Required Toolboxes:**
  - Reinforcement Learning Toolbox
  - Robotics System Toolbox
  - Simulink (for KUKA model)
  - OPC Toolbox (for PLC communication, optional)
- **Hardware (online mode only):**
  - KUKA KR10 with KVP server enabled
  - 2× RoArm-M2-S with WiFi firmware
  - Siemens S7-1500 PLC with OPC-UA server (optional)

---

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/Masimba249/Multirobot-Adaptive-Control-with-Reinforcement-Learning.git
cd Multirobot-Adaptive-Control-with-Reinforcement-Learning
```

### 2. Open MATLAB and set up paths

```matlab
cd MATLAB
setup   % adds all subfolders to the MATLAB path
```

### 3. Run the full pipeline (synthetic mode — no hardware needed)

```matlab
results = run_full_multirobot_pipeline();
```

### 4. Run in online mode (requires hardware)

```matlab
cfg = struct();
cfg.mode     = 'online';
cfg.kukaIp   = '172.31.17.101';
cfg.kukaPort = 7000;
cfg.roarmIps = {'192.168.1.192', '192.168.1.101'};

results = run_full_multirobot_pipeline(cfg);
```

### 5. Run individual components

```matlab
% KUKA pick-and-place only
cd MATLAB/RL
runKukaPickPlaceRL

% RoArm painting only
runRoarmPaintingRL

% Test KUKA connection
cd MATLAB/KUKA_control
KukaTest_Connection

% RoArm demo
cd MATLAB/Roarm_control
RoarmM2_Demo
```

---

## Configuration

All defaults are set in `fillPipelineDefaults()` inside `run.m`. Override any field:

```matlab
cfg = struct();
cfg.mode           = 'synthetic';     % 'synthetic' or 'online'
cfg.trainKuka      = true;            % Train KUKA pick-place agent
cfg.trainRoarm     = true;            % Train RoArm painting agent
cfg.trainCollision = true;            % Train collision avoidance agent

% RL hyperparameters (collision avoidance)
cfg.collisionCfg.train.maxEpisodes   = 500;
cfg.collisionCfg.train.hiddenSizes   = [128, 128];
cfg.collisionCfg.train.actorLR       = 1e-4;
cfg.collisionCfg.train.criticLR      = 1e-3;
cfg.collisionCfg.train.gamma         = 0.99;
cfg.collisionCfg.train.bufferLength  = 100000;

results = run_full_multirobot_pipeline(cfg);
```

---

## RL Agents

### KUKA Pick-and-Place (TD3)

- **Observation (8-dim):** velocity, acceleration, curvature, tracking error, torque estimate, progress, distance to pick, distance to place
- **Action (1-dim):** velocity override [0, 1]
- **Goal:** minimize cycle time while maintaining smooth trajectories

### RoArm Painting (TD3)

- **Observation (8-dim):** tracking error, velocity, acceleration, curvature, progress, dwell time, previous speed, error delta
- **Action (1-dim):** painting speed command [0, 1]
- **Goal:** uniform paint coverage with minimal overspray

### Collision Avoidance (DDPG)

- **Observation (12-dim):** positions of all 3 robots, pairwise distances, velocities
- **Action (6-dim):** position adjustments for both RoArm units
- **Goal:** maintain safe separation while minimizing deviation from painting targets

### Saved Agents

Over **450 agent checkpoints** are stored in `MATLAB/RL/savedAgents/` as `.mat` files. The pipeline automatically loads the latest checkpoint when resuming.

---

## PLC Integration

The system optionally interfaces with a **Siemens S7-1500 PLC** via OPC-UA for real-time safety monitoring:

```matlab
PLC_OPC_UA_Launcher    % Launch OPC-UA connection and write robot data
```

Data written to PLC includes robot positions, velocities, and collision flags, enabling hardware safety interlocks independent of the MATLAB controller.

---

## Pipeline Output

Each run produces:

| Output | Location |
|--------|----------|
| Pipeline summary struct | `MATLAB/pipeline_results_<timestamp>.mat` |
| Collision dataset CSV | `MATLAB/RL/collision_dataset_<timestamp>.csv` |
| Trained agents | `MATLAB/RL/savedAgents/` |
| Training figures | `MATLAB/RL/figures/` |

Example summary:

```
  Phase                               Status
  --------------------------------------------------
  0. Setup                            PASS
  1. KUKA Pick-Place RL               PASS
  2. RoArm Painting RL                PASS
  3. Live Multi-Robot Collect          PASS
  4. Collision Avoidance RL            PASS

  PASS: 5 | FAIL: 0 | SKIPPED: 0
```

---

## License

This project is licensed under the [MIT License](LICENSE).

---

<p align="center">
  <sub>Built with MATLAB · Reinforcement Learning Toolbox · Robotics System Toolbox</sub>
</p>
