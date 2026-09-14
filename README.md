# Vehicle Efficiency Optimisation
### Final Year Physics Project (University of Nottingham, 2024)

> **Computational optimisation of vehicle driving efficiency across varying terrain profiles using Monte Carlo simulation.**
> Built as a final year project in the School of Physics & Astronomy — directly analogous to route optimisation and energy-efficient fleet navigation in smart systems.

---

## Overview

What is the most efficient way to drive a car? This project answers that question computationally, using a physics engine built from scratch in Python and a Monte Carlo optimisation method to simulate **300,000 journeys** across six terrain profiles.

The study models a **2020 Ford Fiesta 1.0L EcoBoost (125hp, 6-speed manual)** using real torque/power curve data, and determines the optimal throttle variation strategy for minimising energy consumption while maintaining a target average speed.

---

## Motivation & Relevance

This project was originally motivated by real-world fuel economy and emissions reduction. The computational methodology — model vehicle physics, define a control parameter space (throttle variation), and optimise across simulated scenarios — maps directly to:

- **Energy-efficient fleet management** for autonomous vehicle systems  
- **Real-time motion control** where look-ahead terrain data drives actuation decisions

The key finding (anticipate upcoming terrain and use built-up momentum rather than constant throttle) mirrors how modern traffic management systems pre-plan routes to minimise energy and maximise throughput.

---

## Repository Structure

```
vehicle-efficiency-optimisation/
│
├── physics_engine.py          # Core vehicle dynamics model (torque, drag, gears)
├── monte_carlo.py             # Optimisation loop — 300,000 journey simulations
├── terrain_profiles.py        # Six terrain types: Flat, Incline, Decline, FlatTop, FlatBottom, Bumpy
├── results/
│   └── figures/               # Output plots from the simulation
└── report/
    └── final_version.pdf      # Full 40-page project report
```

---

## Physics Engine

The vehicle model accounts for:

| Force | Description |
|---|---|
| **Engine torque** | From real Fiesta power/torque curves interpolated across RPM range |
| **Gear ratios** | All 6 gears modelled; upshift at 3500 RPM, downshift at 1500 RPM |
| **Aerodynamic drag** | F = ½ρCdAv² (standard air resistance model) |
| **Rolling resistance** | Proportional to vehicle weight and road surface coefficient |
| **Gravitational force** | ±mg·sin(θ) across terrain gradient θ |

Time integration uses the **Euler method** with a fixed timestep, yielding velocity and position as functions of time for any given throttle profile.

---

## Optimisation Method

A **Monte Carlo approach** is used:
1. Randomly sample a throttle profile (throttle value per road segment)
2. Run the physics engine to simulate the journey
3. Record total energy consumed and journey completion time
4. After 300,000 iterations, extract the Pareto front — optimal strategies at each speed/energy trade-off

The output is an energy vs. completion time scatter plot; the optimal strategy sits at the minimum energy for a given time constraint.

---

## Key Results

| Terrain | Optimal Strategy |
|---|---|
| **Flat** | Reduce throttle in the final segment — use built-up momentum |
| **Incline** | Build maximum momentum before the hill; carry it over the crest |
| **Decline** | Reduce throttle early; let gravity do the work |
| **Composite** | The combined optimal ≠ sum of individual strategies — look-ahead is critical |
| **Bumpy (random slopes)** | Behaves like flat — random gradients average to zero |

---

## Requirements

```bash
pip install numpy matplotlib
```

No external physics libraries — the engine is implemented from scratch using NumPy.

---

## Running the Simulation

```bash
# Run the Monte Carlo optimisation for the flat terrain profile
python monte_carlo.py --terrain flat --journeys 10000 --segments 8 --speed 50

# Run the full physics engine for a single throttle profile
python physics_engine.py
```

---

## Authors

Miguel PJ Arias · University of Nottingham, School of Physics & Astronomy · May 2024  
Ethan White · University of Nottingham, School of Physics & Astronomy · May 2024  
*Pair project — vehicle physics engine and optimisation framework built collaboratively.*

---

## Report

The full 40-page project report is included at `report/final_version.pdf`. It covers the complete theoretical framework, validation tests, and results across all six terrain profiles with uncertainty analysis.
