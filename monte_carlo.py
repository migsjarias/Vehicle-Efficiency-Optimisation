"""
monte_carlo.py
==============
Monte Carlo optimisation for vehicle driving efficiency.

Randomly samples throttle profiles across a road divided into segments,
simulates each journey using the physics engine, and records energy
consumption vs. completion time. The result is a scatter plot (Pareto front)
showing the optimal energy-time trade-off.

The optimal strategy sits at the minimum energy for a given time constraint —
this is the "most efficient" way to drive for a given average speed target.

Usage (CLI)
-----------
    python monte_carlo.py --terrain flat --journeys 10000 --segments 8 --speed 50

    Options:
        --terrain     Terrain profile name (flat/incline/decline/flat_top/flat_bottom/bumpy)
        --journeys    Number of Monte Carlo iterations (default: 10000)
        --segments    Number of road segments (default: 8)
        --speed       Target average speed in mph (default: 50)
        --distance    Road distance in miles (default: 10)

Authors: Miguel PJ Arias — University of Nottingham, 2024
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from physics_engine import simulate_journey
from terrain_profiles import get_profile


# ---------------------------------------------------------------------------
# MONTE CARLO CORE
# ---------------------------------------------------------------------------

def run_monte_carlo(
    terrain_name: str,
    n_journeys: int,
    n_segments: int,
    target_speed_mph: float,
    road_distance_miles: float,
    seed: int = None
) -> dict:
    """
    Run the Monte Carlo optimisation.

    For each iteration, a random throttle profile is sampled and a full
    journey simulated. Results are collected for all journeys that complete
    within a reasonable time envelope around the target average speed.

    Parameters
    ----------
    terrain_name : str
        Name of terrain profile (see terrain_profiles.py).
    n_journeys : int
        Number of Monte Carlo iterations.
    n_segments : int
        Number of road segments (throttle is constant per segment).
    target_speed_mph : float
        Target average speed in mph — used to define a time tolerance window.
    road_distance_miles : float
        Total road distance in miles.
    seed : int, optional
        Random seed for reproducibility.

    Returns
    -------
    dict with keys:
        energies    : np.ndarray of energy values (kWh) for valid journeys
        times       : np.ndarray of completion times (s) for valid journeys
        throttles   : list of throttle profiles for valid journeys
        terrain     : np.ndarray of gradient values used
        n_valid     : number of journeys within the speed tolerance window
    """
    rng = np.random.default_rng(seed)

    road_distance_m = road_distance_miles * 1609.34
    segment_length_m = road_distance_m / n_segments

    # Time window: ±20% of the target journey time
    target_time_s = (road_distance_miles / target_speed_mph) * 3600
    time_min = target_time_s * 0.80
    time_max = target_time_s * 1.20

    terrain = get_profile(terrain_name, n_segments, seed=seed)

    energies = []
    times = []
    throttles = []

    print(f"\nRunning {n_journeys:,} Monte Carlo simulations...")
    print(f"  Terrain:        {terrain_name}")
    print(f"  Segments:       {n_segments}")
    print(f"  Distance:       {road_distance_miles} miles")
    print(f"  Target speed:   {target_speed_mph} mph")
    print(f"  Time window:    {time_min:.0f}s – {time_max:.0f}s\n")

    for i in range(n_journeys):
        if i % (n_journeys // 10) == 0:
            print(f"  Progress: {100 * i // n_journeys}%")

        # Sample random throttle profile — values between 0.1 and 1.0
        throttle = rng.uniform(0.1, 1.0, n_segments)

        result = simulate_journey(throttle, terrain, segment_length_m)

        # Only keep journeys within the speed tolerance window
        if time_min <= result["total_time_s"] <= time_max:
            energies.append(result["total_energy_kwh"])
            times.append(result["total_time_s"])
            throttles.append(throttle)

    n_valid = len(energies)
    print(f"\n  Complete. {n_valid:,} valid journeys ({100*n_valid//n_journeys}% hit rate).")

    return {
        "energies":  np.array(energies),
        "times":     np.array(times),
        "throttles": throttles,
        "terrain":   terrain,
        "n_valid":   n_valid,
    }


def find_optimal(results: dict) -> tuple:
    """
    Returns the throttle profile with the minimum energy consumption.

    Parameters
    ----------
    results : dict
        Output from run_monte_carlo().

    Returns
    -------
    tuple : (min_energy_kwh, completion_time_s, optimal_throttle_profile)
    """
    if results["n_valid"] == 0:
        raise ValueError("No valid journeys found — widen the speed tolerance or reduce target speed.")

    best_idx = np.argmin(results["energies"])
    return (
        results["energies"][best_idx],
        results["times"][best_idx],
        results["throttles"][best_idx]
    )


# ---------------------------------------------------------------------------
# PLOTTING
# ---------------------------------------------------------------------------

def plot_results(results: dict, terrain_name: str, target_speed_mph: float):
    """Scatter plot of energy vs completion time, with optimal point highlighted."""

    energies = results["energies"]
    times = results["times"]

    if len(energies) == 0:
        print("No valid journeys to plot.")
        return

    min_energy, best_time, best_throttle = find_optimal(results)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(
        f"Monte Carlo Optimisation — {terrain_name.replace('_',' ').title()} Terrain "
        f"@ {target_speed_mph} mph avg",
        fontsize=13
    )

    # Scatter: energy vs time
    scatter = ax1.scatter(
        times / 60, energies,
        c=energies, cmap="plasma_r",
        s=4, alpha=0.5, linewidths=0
    )
    ax1.scatter(best_time / 60, min_energy,
                color="lime", s=80, zorder=5,
                edgecolors="black", linewidths=0.8,
                label=f"Optimal: {min_energy:.4f} kWh")
    ax1.set_xlabel("Journey Time (min)")
    ax1.set_ylabel("Energy Consumed (kWh)")
    ax1.legend(fontsize=9)
    ax1.grid(alpha=0.3)
    plt.colorbar(scatter, ax=ax1, label="Energy (kWh)")

    # Bar chart of optimal throttle profile
    n = len(best_throttle)
    ax2.bar(range(1, n + 1), best_throttle * 100,
            color="steelblue", edgecolor="k", linewidth=0.5)
    ax2.set_xlabel("Road Segment")
    ax2.set_ylabel("Throttle (%)")
    ax2.set_title("Optimal Throttle Profile")
    ax2.set_ylim(0, 105)
    ax2.set_xticks(range(1, n + 1))
    ax2.grid(axis="y", alpha=0.4)

    plt.tight_layout()
    output_path = f"results/{terrain_name}_{int(target_speed_mph)}mph.png"
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved: {output_path}")
    plt.show()


# ---------------------------------------------------------------------------
# CLI ENTRY POINT
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Monte Carlo vehicle efficiency optimisation"
    )
    parser.add_argument("--terrain",   type=str,   default="flat",
                        help="Terrain profile name")
    parser.add_argument("--journeys",  type=int,   default=10_000,
                        help="Number of Monte Carlo iterations")
    parser.add_argument("--segments",  type=int,   default=8,
                        help="Number of road segments")
    parser.add_argument("--speed",     type=float, default=50.0,
                        help="Target average speed (mph)")
    parser.add_argument("--distance",  type=float, default=10.0,
                        help="Road length (miles)")
    parser.add_argument("--seed",      type=int,   default=None,
                        help="Random seed for reproducibility")
    args = parser.parse_args()

    results = run_monte_carlo(
        terrain_name=args.terrain,
        n_journeys=args.journeys,
        n_segments=args.segments,
        target_speed_mph=args.speed,
        road_distance_miles=args.distance,
        seed=args.seed
    )

    if results["n_valid"] > 0:
        min_energy, best_time, best_throttle = find_optimal(results)
        print(f"\nOptimal journey:")
        print(f"  Energy:         {min_energy:.4f} kWh")
        print(f"  Time:           {best_time:.1f} s ({best_time/60:.1f} min)")
        print(f"  Throttle:       {[f'{t:.2f}' for t in best_throttle]}")

    plot_results(results, args.terrain, args.speed)
