"""
physics_engine.py
=================
Vehicle dynamics model for the Ford Fiesta 1.0L EcoBoost (125hp, 6-speed manual).

Simulates a car journey along a 2D road profile, computing velocity, RPM,
gear position, and cumulative energy consumption at each timestep using
Euler integration.

Inputs:  throttle profile (array of throttle values per road segment),
         terrain profile (array of gradient angles in degrees),
         target average speed (mph)

Outputs: velocity vs time, energy consumption, gear trace, RPM trace

Authors: Miguel PJ Arias — University of Nottingham, 2024
"""

import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# VEHICLE CONSTANTS — 2020 Ford Fiesta 1.0L EcoBoost
# ---------------------------------------------------------------------------

MASS = 1200             # Vehicle mass (kg)
CD = 0.30               # Drag coefficient
FRONTAL_AREA = 2.2      # Frontal area (m^2)
RHO_AIR = 1.225         # Air density at sea level (kg/m^3)
CRR = 0.015             # Rolling resistance coefficient
G = 9.81                # Gravitational acceleration (m/s^2)
DRIVETRAIN_EFF = 0.85   # Drivetrain power loss (~15% is standard)

# Gear ratios (individual) and final drive ratio
GEAR_RATIOS = [3.417, 1.958, 1.276, 0.943, 0.757, 0.634]
FINAL_DRIVE = 3.941
OVERALL_RATIOS = [g * FINAL_DRIVE for g in GEAR_RATIOS]

# Gear change thresholds (RPM)
UPSHIFT_RPM = 3500
DOWNSHIFT_RPM = 1500
REDLINE_RPM = 6500
IDLE_RPM = 1000

# Wheel radius (m) — standard 195/55 R15 tyre
WHEEL_RADIUS = 0.299

# Simulation timestep
DT = 0.1  # seconds


# ---------------------------------------------------------------------------
# TORQUE CURVE — interpolated from manufacturer data
# ---------------------------------------------------------------------------

# RPM breakpoints and corresponding torque values (Nm) from power/torque curve
TORQUE_RPM_POINTS = np.array([1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500])
TORQUE_NM_POINTS  = np.array([100,  170,  200,  210,  215,  200,  190,  180,  165,  150,  130,  100])


def engine_torque(rpm: float, throttle: float) -> float:
    """
    Returns engine torque (Nm) at a given RPM and throttle position.

    Throttle scales the torque curve linearly between 0 (no power)
    and 1 (wide open throttle).

    Parameters
    ----------
    rpm : float
        Current engine RPM
    throttle : float
        Throttle position in range [0, 1]

    Returns
    -------
    float
        Net torque at the wheels in Nm
    """
    rpm = np.clip(rpm, IDLE_RPM, REDLINE_RPM)
    peak_torque = np.interp(rpm, TORQUE_RPM_POINTS, TORQUE_NM_POINTS)
    return throttle * peak_torque


def select_gear(rpm: float, current_gear: int) -> int:
    """
    Determines gear position based on current RPM and shift thresholds.

    Parameters
    ----------
    rpm : float
        Current engine RPM
    current_gear : int
        Current gear (1-indexed, range 1–6)

    Returns
    -------
    int
        New gear position after applying shift logic
    """
    if rpm >= UPSHIFT_RPM and current_gear < 6:
        return current_gear + 1
    elif rpm <= DOWNSHIFT_RPM and current_gear > 1:
        return current_gear - 1
    return current_gear


def velocity_to_rpm(velocity_ms: float, gear: int) -> float:
    """
    Converts vehicle speed (m/s) to engine RPM for a given gear.

    Parameters
    ----------
    velocity_ms : float
        Vehicle speed in m/s
    gear : int
        Current gear (1-indexed)

    Returns
    -------
    float
        Engine RPM
    """
    overall_ratio = OVERALL_RATIOS[gear - 1]
    wheel_angular_velocity = velocity_ms / WHEEL_RADIUS   # rad/s
    engine_angular_velocity = wheel_angular_velocity * overall_ratio
    rpm = engine_angular_velocity * 60 / (2 * np.pi)
    return max(rpm, IDLE_RPM)


# ---------------------------------------------------------------------------
# RESISTIVE FORCES
# ---------------------------------------------------------------------------

def aerodynamic_drag(velocity_ms: float) -> float:
    """Aerodynamic drag force (N) at a given speed."""
    return 0.5 * RHO_AIR * CD * FRONTAL_AREA * velocity_ms**2


def rolling_resistance(gradient_deg: float = 0.0) -> float:
    """Rolling resistance force (N), accounting for road gradient."""
    return CRR * MASS * G * np.cos(np.radians(gradient_deg))


def gradient_force(gradient_deg: float) -> float:
    """Gravitational component along the road (N). Positive = uphill resistance."""
    return MASS * G * np.sin(np.radians(gradient_deg))


# ---------------------------------------------------------------------------
# EULER INTEGRATION — SINGLE JOURNEY SIMULATION
# ---------------------------------------------------------------------------

def simulate_journey(
    throttle_profile: np.ndarray,
    terrain_profile: np.ndarray,
    segment_length_m: float,
    dt: float = DT
) -> dict:
    """
    Simulates a full vehicle journey using Euler integration.

    The road is divided into segments. The throttle value and terrain gradient
    are constant within each segment. The simulation steps through time,
    updating velocity using Newton's second law at each step.

    Parameters
    ----------
    throttle_profile : np.ndarray
        Throttle value (0–1) for each road segment.
    terrain_profile : np.ndarray
        Gradient angle (degrees) for each road segment. Positive = uphill.
    segment_length_m : float
        Length of each road segment in metres.
    dt : float
        Simulation timestep in seconds.

    Returns
    -------
    dict with keys:
        time        : time array (s)
        velocity    : velocity array (m/s)
        rpm         : engine RPM array
        gear        : gear position array
        energy_kwh  : cumulative energy consumed (kWh)
        total_time  : journey completion time (s)
        total_energy: total energy consumed (kWh)
    """
    n_segments = len(throttle_profile)
    total_distance = n_segments * segment_length_m

    # State initialisation
    velocity = 0.0          # m/s
    position = 0.0          # m
    gear = 1
    energy_joules = 0.0
    time_elapsed = 0.0

    # Logging arrays
    time_log = []
    velocity_log = []
    rpm_log = []
    gear_log = []
    energy_log = []

    while position < total_distance:
        # Determine which segment we're in
        seg_idx = min(int(position / segment_length_m), n_segments - 1)
        throttle = throttle_profile[seg_idx]
        gradient = terrain_profile[seg_idx]

        # Current RPM and gear
        rpm = velocity_to_rpm(velocity, gear)
        gear = select_gear(rpm, gear)
        rpm = velocity_to_rpm(velocity, gear)  # recalculate after shift

        # Wheel torque and tractive force
        torque_engine = engine_torque(rpm, throttle)
        torque_wheel = torque_engine * OVERALL_RATIOS[gear - 1] * DRIVETRAIN_EFF
        tractive_force = torque_wheel / WHEEL_RADIUS

        # Resistive forces
        f_drag = aerodynamic_drag(velocity)
        f_roll = rolling_resistance(gradient)
        f_grad = gradient_force(gradient)

        # Net force and acceleration
        net_force = tractive_force - f_drag - f_roll - f_grad
        acceleration = net_force / MASS

        # Euler integration step
        velocity = max(velocity + acceleration * dt, 0.0)
        position += velocity * dt
        time_elapsed += dt

        # Energy: power = force × velocity, energy = power × dt
        power_w = tractive_force * velocity
        energy_joules += power_w * dt

        # Log state
        time_log.append(time_elapsed)
        velocity_log.append(velocity * 2.237)   # convert to mph for output
        rpm_log.append(rpm)
        gear_log.append(gear)
        energy_log.append(energy_joules / 3.6e6)  # convert to kWh

    return {
        "time":         np.array(time_log),
        "velocity_mph": np.array(velocity_log),
        "rpm":          np.array(rpm_log),
        "gear":         np.array(gear_log),
        "energy_kwh":   np.array(energy_log),
        "total_time_s": time_elapsed,
        "total_energy_kwh": energy_joules / 3.6e6,
    }


# ---------------------------------------------------------------------------
# EXAMPLE — Full throttle benchmark run
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Flat 10-mile road, 8 segments, full throttle throughout
    n_seg = 8
    road_length_m = 10 * 1609.34         # 10 miles in metres
    seg_length_m = road_length_m / n_seg

    throttle = np.ones(n_seg)            # Full throttle
    terrain  = np.zeros(n_seg)           # Flat road

    result = simulate_journey(throttle, terrain, seg_length_m)

    print(f"Journey complete.")
    print(f"  Total time:   {result['total_time_s']:.1f} s "
          f"({result['total_time_s']/60:.1f} min)")
    print(f"  Total energy: {result['total_energy_kwh']:.4f} kWh")

    # Plot velocity and gear traces
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Full Throttle Benchmark — Flat 10-Mile Road", fontsize=13)

    ax1.plot(result["time"], result["velocity_mph"], color="royalblue")
    ax1.set_ylabel("Speed (mph)")
    ax1.grid(alpha=0.4)

    ax2.plot(result["time"], result["rpm"], color="darkorange")
    ax2.axhline(UPSHIFT_RPM, color="red", linestyle="--", linewidth=0.8, label="Upshift threshold")
    ax2.set_ylabel("Engine RPM")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.4)

    ax3.plot(result["time"], result["gear"], color="green", drawstyle="steps-post")
    ax3.set_ylabel("Gear")
    ax3.set_xlabel("Time (s)")
    ax3.set_yticks(range(1, 7))
    ax3.grid(alpha=0.4)

    plt.tight_layout()
    plt.savefig("results/full_throttle_benchmark.png", dpi=150, bbox_inches="tight")
    plt.show()
