# Vehicle Efficiency Optimisation
### Final Year Physics Project (University of Nottingham, 2024)

> **Computational optimisation of vehicle driving efficiency across varying terrain profiles using Monte Carlo simulation.**
> Built as a final year project in the School of Physics & Astronomy. The method — parameterised control, sampled trade space, Pareto front recovery, validation against an analytical limit — is the one used for low-thrust trajectory optimisation and Δv–time-of-flight trade studies.

---

## Overview

What is the most efficient way to drive a car? This project attempts to answer that question computationally, using a physics engine built from scratch in Python and a Monte Carlo optimisation method to simulate **300,000 journeys** across six terrain profiles.

The study models a **2020 Ford Fiesta 1.0L EcoBoost (125hp, 6-speed manual)** using real torque/power curve data, and determines the optimal throttle variation strategy for minimising energy consumption while maintaining a target average speed.

---

## Motivation & Relevance

This project was originally motivated by real-world fuel economy and emissions reduction. The methodology: model vehicle physics, define a control parameter space (throttle variation), and optimise across simulated scenarios.

This can map directly to:
- **Δv–time-of-flight trade studies** — the energy vs. journey-time Pareto front is the same object as a porkchop plot: cost against duration, pick your operating point
- **Low-thrust trajectory optimisation** — discretising throttle per road segment is the same parameterisation as thrust-per-arc in direct transcription methods
- **Energy-efficient fleet management** for autonomous vehicle systems

The key finding (anticipate upcoming terrain and use built-up momentum rather than constant throttle) mirrors how modern traffic management systems pre-plan routes to minimise energy and maximise throughput.

---

## Repository Structure

```
vehicle-efficiency-optimisation/
│
├── physics_engine.py          # Core vehicle dynamics model (torque, drag, gears)
├── monte_carlo.py             # Optimisation loop: 300,000 journey simulations
├── terrain_profiles.py        # Six terrain types: Flat, Incline, Decline, Flat Top, Flat Bottom, Bumpy
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

Time integration uses the Euler method with a fixed timestep, yielding velocity and position as functions of time for any given throttle profile.

---

## Optimisation Method

A **Monte Carlo approach** is used:
1. Randomly sample a throttle profile (throttle value per road segment)
2. Run the physics engine to simulate the journey
3. Record total energy consumed and journey completion time
4. After 300,000 iterations, extract the Pareto front (optimal strategies at each speed/energy trade-off)

The output is an energy vs. completion time scatter plot; the optimal strategy sits at the minimum energy for a given time constraint.

---

## Key Results

| Terrain | Optimal Strategy |
|---|---|
| **Flat** | Reduce throttle in the final segment, i.e. use built-up momentum |
| **Incline** | Build maximum momentum before the hill; carry it over the crest |
| **Decline** | Reduce throttle early; let gravity do the work (fairly straightforward, but had to be sanity checked!) |
| **Composite** | The combined optimal ≠ sum of individual strategies — look-ahead is critical |
| **Bumpy (random slopes)** | Behaves like flat: random gradients average to zero |

---

## Authors

Miguel PJ Arias · School of Physics & Astronomy, University of Nottingham · May 2024
Ethan White · School of Physics & Astronomy, University of Nottingham · May 2024
*Paired project: vehicle physics engine and optimisation framework built collaboratively.*

**Project Supervisor:** Professor Simon Dye, School of Physics & Astronomy, University of Nottingham · May 2024

---

## Report

The full 40-page project report is included in these files. It covers the complete theoretical framework, validation tests, and results across all six terrain profiles with uncertainty analysis.
