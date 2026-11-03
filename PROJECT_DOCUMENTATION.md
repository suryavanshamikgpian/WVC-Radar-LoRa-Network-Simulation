# Wildlife-Vehicle Collision Mitigation — Radar-LoRa Sensor Network Monte Carlo Simulation

## Complete Project Documentation

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Problem Statement](#2-problem-statement)
3. [Proposed Solution Architecture](#3-proposed-solution-architecture)
4. [Code Structure & File Descriptions](#4-code-structure--file-descriptions)
5. [Simulation Engine — Detailed Walkthrough](#5-simulation-engine--detailed-walkthrough)
   - 5.1 [Configuration (`Config` dataclass)](#51-configuration)
   - 5.2 [Entity Definitions](#52-entity-definitions)
   - 5.3 [Animal Behaviour Model (FID-Draw Core)](#53-animal-behaviour-model-fid-draw-core)
   - 5.4 [Vehicle Dynamics (IDM)](#54-vehicle-dynamics-idm)
   - 5.5 [Sensor Detection Pipeline](#55-sensor-detection-pipeline)
   - 5.6 [Magnetometer Vehicle Sensing](#56-magnetometer-vehicle-sensing)
   - 5.7 [LoRa Awareness Propagation](#57-lora-awareness-propagation)
   - 5.8 [Collision Detection](#58-collision-detection)
   - 5.9 [Simulation Loop](#59-simulation-loop)
6. [Monte Carlo Harness & Experimental Design](#6-monte-carlo-harness--experimental-design)
7. [Statistical Analysis Pipeline](#7-statistical-analysis-pipeline)
8. [Sensitivity Sweeps](#8-sensitivity-sweeps)
9. [Web Console (`wvc_web.py`)](#9-web-console)
10. [Inputs & Outputs](#10-inputs--outputs)
11. [Execution Flow Diagram](#11-execution-flow-diagram)
12. [How to Run](#12-how-to-run)
13. [Key Results Summary](#13-key-results-summary)

---

## 1. Project Overview

This project implements a **Monte Carlo simulation framework** that evaluates a proposed **combined radar and magnetometer sensor network with LoRa-mediated awareness propagation** for mitigating **Wildlife-Vehicle Collisions (WVCs)** on rural road corridors. It serves as a **model-based proof of concept** for the sensor architecture described in the associated research paper.

**Author:** Suryavansham Sahoo (IIT Kharagpur)

**Publication:** *"A Radar-Magnetometer Sensor Network with LoRa-Mediated Awareness Propagation for Wildlife-Vehicle Collision Mitigation: A Monte Carlo Simulation-Based Proof of Concept"* — MDPI Sustainability, 2026.

**Core Question:** Can a dense network of Doppler radar nodes (detecting animals) and magnetometer sites (detecting vehicles), coordinated via LoRa radio, significantly reduce wildlife-vehicle collision rates compared to no intervention?

---

## 2. Problem Statement

Wildlife-Vehicle Collisions are a significant ecological and road-safety problem on rural corridors. Animals (particularly deer, elk, and moose) cross roads and are struck by vehicles. The goal is to evaluate — via simulation — whether deploying a sensor network along a road corridor can:

- **Detect approaching animals** before they enter the road
- **Alert drivers** via Dynamic Message Signs (DMS) to reduce speed
- **Propagate awareness** across the sensor network using LoRa so that neighbouring sensors become more sensitive after a detection event

---

## 3. Proposed Solution Architecture

The sensor system consists of three layers:

### Layer 1: Animal Detection (Doppler Radar)
- Alternating-side Doppler radar nodes at **15 m spacing** (7.5 m offset between sides)
- ~134 nodes per km across both road shoulders
- Baseline detection radius: **10 m**
- Detection probability follows an exponential rate model scaled by animal size (RCS proxy)

### Layer 2: Vehicle Sensing (Magnetometers)
- Three-axis magnetometer sites every **200 m**
- Detect vehicles via ferromagnetic mass
- Gate driver alerts on confirmed vehicle presence (no blind corridor-wide alerts)

### Layer 3: LoRa Network Awareness
- On any animal detection, a LoRa broadcast propagates to all radars within **1500 m**
- Receiving radars get a **1.5× sensitivity boost** for **30 seconds** (range extension)
- Dynamic Message Signs (DMS) every 250 m display a caution speed (**30 km/h**)

### Three Simulation Modes

| Mode | Sensors | Driver Alert (DMS) | LoRa Awareness Boost |
|------|:-------:|:------------------:|:-------------------:|
| **Control** | Off | Off | Off |
| **Detection** | Active | Yes | Off |
| **Aware** | Active | Yes | β=1.5× within 1500m for 30s |

---

## 4. Code Structure & File Descriptions

```
WVC-Radar-LoRa-Sensor-Network-MonteCarlo-Simulation/
├── wvc.py                    # Main simulation engine (3309 lines)
│                               Unified CLI with 6 subcommands:
│                               simulate, data, stats, figures, ablation, pipeline
├── compute_stats.py          # Companion statistical analysis (510 lines)
│                               Welch t-tests, GLM, bootstrap, interaction tests
├── wvc_web.py                # Local web console for running simulations (1100 lines)
│                               Browser-based GUI with live terminal streaming
├── requirements.txt          # Python dependencies
├── CITATION.cff              # Citation metadata
├── LICENSE                   # MIT License
├── README.md                 # Original project README
├── PROJECT_DOCUMENTATION.md  # This file — detailed code documentation
├── data/                     # Simulation output data
│   ├── wvc_results.csv                   # Headline per-trial results (60 trials)
│   ├── wvc_results_<param>.csv           # Per-sweep raw results (13 sweeps)
│   ├── wvc_stats_headline.csv            # Headline statistics
│   ├── wvc_stats_<param>.csv             # Per-sweep statistics
│   ├── wvc_stats_<param>_interaction.csv # Treatment × parameter interaction tests
│   ├── regeneration_summary.{json,txt}   # Run manifest
│   └── .regeneration_checkpoint.json     # Resume checkpoint
└── docs/                     # GitHub Pages data page
    ├── index.html            # Interactive data explorer
    └── figure_*.png          # Publication-grade figures (17 figures)
```

---

## 5. Simulation Engine — Detailed Walkthrough

### 5.1 Configuration

The `Config` dataclass (lines 108–236 in `wvc.py`) centralises **every tunable parameter**:

| Category | Key Parameters | Default Values |
|----------|---------------|----------------|
| **Road geometry** | `road_length`, `road_half_width` | 1000 m, 2 m |
| **Traffic** | `vehicles_per_dir`, `cruise_speed_kmh`, `caution_speed_kmh` | 4, 100 km/h, 30 km/h |
| **Wildlife arrival** | `animal_rate_per_hr`, size mixture fractions | λ=15/hr, 15% small, 60% medium, 25% large |
| **Animal behaviour** | `forage_dwell`, `approach_speed`, `cross_speed`, `flee_speed` | 2–10 s, 1.5 m/s, 4 m/s, 6 m/s |
| **FID-draw** | `fid_log_mu`, `fid_log_sigma`, `fid_nonresponder_p` | ln(72), 0.702, 18.8% |
| **Radar sensors** | `radar_spacing`, `radar_range`, `detection_rate_per_sec` | 15 m, 10 m, κ=3.0 |
| **Magnetometers** | `magnetic_spacing`, `mag_pd_r50`, `mag_buffer_s` | 200 m, 4.2 m, 10 s |
| **LoRa/Awareness** | `awareness_range_m`, `awareness_persist_s`, `boost_factor` | 1500 m, 30 s, 1.5× |
| **IDM dynamics** | `idm_a_max`, `idm_b_comf`, `idm_a_emergency` | 2.5, 4.0, 8.0 m/s² |
| **Simulation** | `dt`, `seed` | 0.1 s, None |

### 5.2 Entity Definitions

Four entity types are defined as dataclasses:

- **`Animal`** — Position (x,y), side, size (RCS factor), behavioural state, detection status, FID parameters
- **`Vehicle`** — Position, direction, speed, alert state, crash state, magnetometer sensing state
- **`Radar`** — Position, side, boost expiry time
- **`MagSite`** — Position, side (magnetometer sensor location)

### 5.3 Animal Behaviour Model (FID-Draw Core)

The animal behaviour is governed by a **six-state finite state machine**:

```
FORAGING → APPROACHING → HESITATING → { CROSSING | FROZEN | FLEEING } → GONE
```

**Key mechanism — Flight-Initiation-Distance (FID) draw:**

At spawn, each animal is assigned:
1. **FID distance** — drawn from a log-normal distribution (median 72 m, calibrated to Blackwell et al. 2014 white-tailed-deer field data). 18.8% of animals are **non-responders** (FID = 0, meaning they never flee from vehicles and always cross).
2. **Detection ceiling** — maximum distance at which the animal can perceive a vehicle (log-normal, median 183 m). The effective FID is capped at this ceiling: `effective_FID = min(FID, detection_ceiling)`.

**State transitions:**

| Current State | Transition Condition | Next State |
|--------------|---------------------|------------|
| FORAGING | Dwell timer expires (2–10 s random) | APPROACHING |
| APPROACHING | Reaches road verge (|y| < 3 m) | HESITATING |
| APPROACHING | Vehicle within FID (FID-draw model) | FLEEING |
| HESITATING | Vehicle within FID triggers | FLEEING |
| HESITATING | Dwell timer expires, no FID trigger | CROSSING |
| CROSSING | Vehicle very close + fast (p=0.15×θ per frame) | FROZEN |
| CROSSING | Reaches far side | GONE |
| FROZEN | Dwell timer expires | CROSSING (50%) or FLEEING (50%) |
| FLEEING | Reaches safe distance | GONE |

**Cross-check:** The emergent control-mode flee fraction (0.83) matches the field response rate (~0.81) — the model is untuned against this metric, so the match is a robustness indicator.

### 5.4 Vehicle Dynamics (IDM)

Vehicles follow the **Treiber-Hennecke-Helbing Intelligent Driver Model (IDM)**:

- **Free-flow:** Cruise at v₀ = 100 km/h
- **Under alert:** Target speed drops to v₁ = 30 km/h after a **1.5 s perception-reaction lag**
- **Emergency braking:** Maximum deceleration 8.0 m/s² (dry-asphalt traction ceiling)
- **Car-following:** IDM gap-maintenance with parameters s₀=5 m, T=1.5 s, a_max=2.5 m/s², b_comf=4.0 m/s²
- **Hard-brake:** When a detected animal is on the road ahead, a virtual stopped obstacle is placed for emergency deceleration

**Vehicle wrapping:** When a vehicle exits the corridor, it wraps around (re-entering from the other end at cruise speed) to maintain constant traffic density.

### 5.5 Sensor Detection Pipeline

```
For each radar node:
    For each alive, undetected animal:
        1. Compute distance to radar (2D: dx² + dy²)
        2. Apply awareness boost to effective range: R_eff = R_base × boost_factor
        3. If animal within R_eff:
            - Record "first in range" timestamp
            - Compute size-dependent detection rate: κ_eff = κ × size_factor
            - Per-frame detection probability: P = 1 - exp(-κ_eff × dt)
            - Monte Carlo draw: if random() < P → DETECTED
        4. On detection:
            - Mark animal as detected, record latency
            - Activate awareness (DMS alerts)
            - If "aware" mode: LoRa-broadcast boost to all radars within 1500 m
```

### 5.6 Magnetometer Vehicle Sensing

Two vehicle-sensing models are supported:

- **`perfect`** (default): System has omniscient vehicle knowledge — all vehicles are always alertable
- **`magnetometer`**: Alerts are gated on magnetometer detections
  - Logistic detection probability vs. closest-approach distance
  - Vehicle is "present-known" for `mag_buffer_s` seconds after last detection
  - Alerts are only issued to vehicles that the system knows about

The magnetometer ablation study confirms that the magnetometer model is statistically indistinguishable from the perfect-knowledge oracle (Welch t = +0.25 on collisions/trial).

### 5.7 LoRa Awareness Propagation

Only active in **"aware" mode**:

1. On any animal detection, a LoRa notification propagates to all radars within `awareness_range_m` (1500 m)
2. Receiving radars get `boost_factor` (1.5×) applied to their **detection range** (not detection rate) for `awareness_persist_s` (30 s)
3. This simulates elevated transmit power on the Doppler RF front-end — by the radar equation, R_max ∝ P_t^(1/4)
4. The boost enables earlier detection of **subsequent** animals and animals in coverage gaps

### 5.8 Collision Detection

Each simulation frame checks for collisions:

```python
For each vehicle (not already crashed):
    For each animal (alive, on road):
        If |animal.x - vehicle.x| < 4 m:     # longitudinal proximity
            If lane match (directional):       # lateral band check
                → Animal dies, vehicle crashes
                → Collision counter increments
```

Lane matching uses directional bands:
- Vehicle direction +1 (southbound): collision if -2.2 < animal.y < 0.2
- Vehicle direction -1 (northbound): collision if -0.2 < animal.y < 2.2

### 5.9 Simulation Loop

Each `step()` call (Δt = 0.1 s):

```
1. Advance time by dt
2. Track awareness-active seconds
3. Poisson spawn: accumulate (λ/3600 × dt); spawn animal per unit
4. For each alive animal:
   a. Update behavioural state (state machine + FID trigger)
   b. Move animal (kinematics + random walk noise)
5. Garbage-collect terminal/dead animals
6. Process radar detections (sensor pipeline)
7. Record previous vehicle positions
8. Update vehicles (IDM + alert response)
9. Update magnetometer vehicle sensing
10. Check for collisions
```

---

## 6. Monte Carlo Harness & Experimental Design

### Common Random Numbers (CRN)

All three modes within a single trial share the **same seed**: `seed = trial × 10000`. This ensures:
- Identical animal arrival times, sizes, FID draws, and positions across modes
- Any difference in collision rate reflects the **policy** (sensor configuration), not stochastic world variation
- This is the standard variance-reduction technique for matched-pair policy comparison

### Headline Experiment
- **20 trials × 4 simulated hours per mode × 3 modes = 60 trials**
- 1 km corridor with default configuration

### Sweep Experiments
- **15 trials × 2 hours per (mode, value) × 3 modes** for each sweep
- 13 parameters swept, each across 4–7 values

### Parallelisation
- Trials are distributed across worker processes via `multiprocessing.Pool`
- `imap_unordered` for load balancing
- `_simulate_trial()` is module-level for picklability

---

## 7. Statistical Analysis Pipeline

`compute_stats.py` computes, for **each metric** (collision rate, detection rate, latency) and **each pairwise mode comparison**:

### Tests Performed

| Test | Purpose | Details |
|------|---------|---------|
| **Welch's t-test** | Primary cell-level significance test | Unequal-variance two-sample t-test on per-trial rates |
| **Poisson GLM** | Rate ratio with 95% Wald CI | `collisions ~ C(mode)` with `log(entries)` offset; falls back to Negative Binomial if Pearson dispersion > 1.5 |
| **Bootstrap** | Non-parametric confidence intervals | 10,000-replicate resample of per-trial rate differences |
| **Interaction test** | Mode × sweep parameter interaction | Likelihood-ratio test on nested GLMs (Poisson/NB for counts; two-way OLS ANOVA for latency) |

### Three Metrics

1. **`col_rate`** — Collisions per road entry (primary safety outcome)
2. **`det_rate`** — Detections per arriving animal (sensor performance)
3. **`latency`** — In-range detection latency in seconds (sensor responsiveness)

### Three Pairwise Comparisons

1. Control vs. Detection
2. Control vs. Aware
3. Detection vs. Aware

---

## 8. Sensitivity Sweeps

13 parameters are swept to assess robustness:

| Sweep Label | Config Parameter | Values Swept |
|------------|-----------------|-------------|
| `spacing` | `radar_spacing` | 5, 10, 15, 20, 25, 30, 40 m |
| `radar_range` | `radar_range` | 8, 10, 12, 15, 20 m |
| `size` | `size_scale` | 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0 |
| `detection_rate` | `detection_rate_per_sec` | 0.3, 0.5, 1.0, 2.0, 3.0, 5.0 |
| `awareness_beta` | `boost_factor` | 1.0, 1.1, 1.2, 1.3, 1.4, 1.5 |
| `awareness_tau` | `awareness_persist_s` | 10, 30, 60, 120 s |
| `hesitate_dwell` | `hesitate_dwell_max` | 1.0, 2.0, 3.0, 5.0, 8.0 s |
| `freeze_mid_cross` | `p_freeze_during_cross_base` | 0.0, 0.05, 0.15, 0.30, 0.50 |
| `traffic_volume` | `vehicles_per_dir` | 1, 2, 4, 8, 16 |
| `animal_density` | `animal_rate_per_hr` | 5, 15, 30, 60, 120, 240 |
| `driver_reaction` | `driver_reaction_s` | 0.5, 1.0, 1.5, 2.5, 4.0 s |
| `cruise_speed` | `cruise_speed_kmh` | 60, 80, 100, 120 km/h |
| `caution_speed` | `caution_speed_kmh` | 30, 45, 60, 80, 100 km/h |

**Robustness finding:** 12 of 13 sweeps show **no detectable mode × parameter interaction** (interaction p > 0.7). Traffic volume is the sole exception.

---

## 9. Web Console

`wvc_web.py` provides a **browser-based control console** (dependency-free, standard-library-only HTTP server):

- **Parameter form** — Tune every configuration parameter via a GUI
- **Live terminal** — Server-Sent Events stream the subprocess output in real-time
- **Results panel** — Auto-loads produced CSVs into sortable tables and draws Canvas charts
- **File browser** — Lists and previews output files (CSVs, PNGs)

**Usage:**
```bash
python wvc_web.py               # http://127.0.0.1:8753
python wvc_web.py --port 9000   # custom port
```

---

## 10. Inputs & Outputs

### Inputs

The simulation takes **no external data files** as input. All inputs are parameters configured through:

| Input Method | Description |
|-------------|-------------|
| **`Config` dataclass** | Default values hardcoded (calibrated to Blackwell et al. 2014 field data) |
| **CLI arguments** | Override any parameter via command-line flags (e.g., `--trials 20`, `--hours 4`, `--sweep spacing`) |
| **Web console** | GUI form for all parameters |

**Key configurable inputs:**

| Input | Description | Default |
|-------|-------------|---------|
| Number of trials | Independent Monte Carlo repetitions per mode | 20 (headline), 15 (sweep) |
| Simulation duration | Simulated hours per trial | 4 h (headline), 2 h (sweep) |
| Modes to run | Which of {control, detection, aware} to simulate | All three |
| Sweep parameter | Which parameter to vary | None (single-point) |
| Sweep values | Values to test for the swept parameter | Presets per parameter |
| Random seed | For reproducibility | None (random) for single; deterministic per trial |

### Outputs

| Output File | Content | Format |
|------------|---------|--------|
| `wvc_results.csv` | Per-trial headline results (60 rows: 20 trials × 3 modes) | CSV |
| `wvc_results_<param>.csv` | Per-trial sweep results | CSV |
| `wvc_stats_headline.csv` | Pairwise statistical comparisons (Welch, GLM, bootstrap) | CSV |
| `wvc_stats_<param>.csv` | Per-cell sweep statistics | CSV |
| `wvc_stats_<param>_interaction.csv` | Mode × parameter interaction test results | CSV |
| `wvc_results.png` | 4-panel matplotlib figure (single-point run) | PNG |
| `docs/figure_*.png` | Publication-grade figures (17 total) | PNG 300 DPI |
| `docs/index.html` | Interactive GitHub Pages data explorer | HTML |
| `regeneration_summary.json` | Complete run manifest with all statistics | JSON |
| `regeneration_summary.txt` | Human-readable run summary | TXT |

### Per-Trial CSV Columns

Each row in `wvc_results.csv` contains:

| Column | Description |
|--------|-------------|
| `mode` | Simulation mode: control, detection, or aware |
| `trial` | Trial index (0-based) |
| `animals` | Total animals spawned |
| `detected` | Animals detected by radar |
| `det_rate` | Detection rate (detected / animals) |
| `road_entries` | Animals that entered the road |
| `collisions` | Number of collisions |
| `col_rate` | Collision rate (collisions / road_entries) |
| `warnings` | Number of detection warnings issued |
| `vehicles_alerted` | Vehicles that received alerts |
| `mean_latency_s` | Mean spawn-to-detection latency |
| `in_range_latency_s` | Mean in-range-to-detection latency |
| `awareness_pct` | % of simulation time awareness was active |
| `veh_present_known_pct` | % of vehicle-frames with known vehicle presence |
| `veh_coverage_pct` | % of vehicles ever detected by magnetometer |
| `veh_gated_out_pct` | % of frames where alerts were gated out (vehicle not known) |
| `frozen_on_road_s` | Total seconds animals were frozen on road |
| `n_foraging` ... `n_fleeing` | State visit counts |
| `wall_seconds` | Real-time wall-clock seconds for this trial |

---

## 11. Execution Flow Diagram

### High-Level Pipeline

```
┌─────────────┐      ┌──────────────┐      ┌────────────────┐      ┌──────────────┐
│  wvc.py     │      │  wvc.py      │      │compute_stats.py│      │  wvc_web.py  │
│  data       │─────▶│  stats       │─────▶│  (standalone)  │      │  figures     │
│  (simulate) │      │  (compute)   │      │                │      │  (render)    │
└─────────────┘      └──────────────┘      └────────────────┘      └──────────────┘
       │                     │                      │                       │
       ▼                     ▼                      ▼                       ▼
  wvc_results*.csv     wvc_stats*.csv         Same stats CSVs        docs/figure_*.png
                                                                     docs/index.html
```

### Single Trial Execution Flow

```
Config + Seed
     │
     ▼
┌──────────────────────────────────────────┐
│           WVCSimulation.__init__          │
│  • Init radars (alternating sides)       │
│  • Init magnetometers (every 200 m)      │
│  • Init vehicles (IDM, both directions)  │
└──────────────────┬───────────────────────┘
                   │
     ┌─────────────▼─────────────┐
     │    Simulation Loop        │
     │    (36,000 steps/hr)      │
     │                           │
     │  1. Poisson animal spawn  │
     │  2. Animal state machine  │
     │  3. Animal movement       │
     │  4. Radar detection       │
     │  5. LoRa broadcast (aware)│
     │  6. Vehicle IDM update    │
     │  7. Magnetometer sensing  │
     │  8. Collision check       │
     │                           │
     └─────────────┬─────────────┘
                   │
                   ▼
           TrialStats result dict
```

---

## 12. How to Run

### Prerequisites
- Python 3.10+
- Dependencies: `numpy`, `pandas`, `scipy`, `plotly`, `seaborn`

### Installation
```bash
git clone <your-repo-url>
cd WVC-Radar-LoRa-Sensor-Network-MonteCarlo-Simulation
pip install -r requirements.txt
```

### Quick Simulation (Single Point)
```bash
python wvc.py simulate --trials 10 --hours 2 --plot
```

### Full Reproducible Pipeline (Headline + All Sweeps + Stats + Figures)
```bash
python wvc.py pipeline --jobs 8
```

### Individual Steps
```bash
# 1. Generate headline + sweep data
python wvc.py data --jobs 8

# 2. Compute statistics from CSVs
python wvc.py stats

# 3. Generate publication figures
python wvc.py figures --dpi 300

# 4. Magnetometer ablation study
python wvc.py ablation --part all --trials 30 --jobs 8
```

### Web Console
```bash
python wvc_web.py                 # opens http://127.0.0.1:8753
```

### Standalone Statistics (without wvc.py)
```bash
python compute_stats.py --csv-dir data/ --bootstrap-n 10000 --seed 42
```

---

## 13. Key Results Summary

| Mode | Collision Rate / Road Entry | Detection Rate | In-Range Latency | Reduction vs Control |
|------|:-:|:-:|:-:|:-:|
| **Control** | 13.22% | 0% | — | (reference) |
| **Detection** | **7.89%** | 20.85% | 0.313 s | **40.3%** (GLM p = 0.062) |
| **Aware** | 7.89% | 20.85% | 0.313 s | 40.3% (≡ Detection) |

### Four Headline Findings

1. **Operating envelope:** 96.2% collision reduction at 1 veh/dir/hr (p = 0.003); 85.7% at 2 veh/dir/hr (p = 0.023). System is effective for **rural corridors below ~2 veh/dir/hr**.

2. **Awareness layer bottleneck:** LoRa boost produces a highly significant sensing-side gain (detection-rate interaction p ≈ 10⁻¹⁸) that does **not** propagate to collision rate (interaction p = 0.949). The bottleneck is at the alert-delivery layer, not the sensor layer.

3. **Magnetometer ablation:** Magnetometer-gated vehicle confirmation is statistically indistinguishable from a perfect-knowledge oracle (Welch t = +0.25).

4. **Robustness:** Across 13 one-dimensional sensitivity sweeps, 12 show no detectable mode × parameter interaction (interaction p > 0.7). Traffic volume is the sole exception.

---

*Generated as part of project documentation. See the original [README.md](README.md) for citation information and related work.*
