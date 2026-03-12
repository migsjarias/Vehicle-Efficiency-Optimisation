"""
terrain_profiles.py
===================
Defines the six road terrain profiles used in the Monte Carlo optimisation study.

Each profile returns a NumPy array of gradient angles (in degrees) for
a road divided into n_segments equal-length segments.

Profiles
--------
    flat        : constant zero gradient
    incline     : uniform uphill slope
    decline     : uniform downhill slope
    flat_top    : uphill then downhill (hill shape)
    flat_bottom : downhill then uphill (valley shape)
    bumpy       : random slopes drawn from U(-3°, 3°) — simulates a realistic road

Authors: Miguel PJ Arias — University of Nottingham, 2024
"""

import numpy as np


def flat(n_segments: int) -> np.ndarray:
    """
    Completely flat road.

    Parameters
    ----------
    n_segments : int
        Number of road segments.

    Returns
    -------
    np.ndarray
        Array of zeros (degrees).
    """
    return np.zeros(n_segments)


def incline(n_segments: int, angle_deg: float = 3.0) -> np.ndarray:
    """
    Uniform uphill gradient throughout the journey.

    Parameters
    ----------
    n_segments : int
        Number of road segments.
    angle_deg : float
        Gradient angle in degrees (positive = uphill). Default: 3°.

    Returns
    -------
    np.ndarray
        Array of constant positive gradient values.
    """
    return np.full(n_segments, angle_deg)


def decline(n_segments: int, angle_deg: float = 3.0) -> np.ndarray:
    """
    Uniform downhill gradient throughout the journey.

    Parameters
    ----------
    n_segments : int
        Number of road segments.
    angle_deg : float
        Gradient angle magnitude in degrees. Default: 3°.

    Returns
    -------
    np.ndarray
        Array of constant negative gradient values.
    """
    return np.full(n_segments, -angle_deg)


def flat_top(n_segments: int, angle_deg: float = 3.0) -> np.ndarray:
    """
    Hill profile — uphill for the first half, downhill for the second half.

    Simulates a journey over a single hill where the driver must decide
    how to manage momentum approaching the crest.

    Parameters
    ----------
    n_segments : int
        Number of road segments (should be even for symmetry).
    angle_deg : float
        Gradient angle in degrees. Default: 3°.

    Returns
    -------
    np.ndarray
        Gradient array: positive then negative.
    """
    half = n_segments // 2
    return np.concatenate([
        np.full(half, angle_deg),
        np.full(n_segments - half, -angle_deg)
    ])


def flat_bottom(n_segments: int, angle_deg: float = 3.0) -> np.ndarray:
    """
    Valley profile — downhill for the first half, uphill for the second half.

    Simulates a journey into and out of a valley.

    Parameters
    ----------
    n_segments : int
        Number of road segments (should be even for symmetry).
    angle_deg : float
        Gradient angle in degrees. Default: 3°.

    Returns
    -------
    np.ndarray
        Gradient array: negative then positive.
    """
    half = n_segments // 2
    return np.concatenate([
        np.full(half, -angle_deg),
        np.full(n_segments - half, angle_deg)
    ])


def bumpy(n_segments: int, max_angle_deg: float = 3.0, seed: int = None) -> np.ndarray:
    """
    Realistic road with random gradient variations.

    Slopes are drawn from a uniform distribution U(-max_angle, +max_angle),
    simulating real-world road irregularities. Because the distribution is
    centred at zero, the average gradient is approximately flat, and results
    closely resemble the flat profile on long journeys.

    Parameters
    ----------
    n_segments : int
        Number of road segments.
    max_angle_deg : float
        Maximum gradient magnitude in degrees. Default: 3°.
    seed : int, optional
        Random seed for reproducibility.

    Returns
    -------
    np.ndarray
        Array of random gradient values in [-max_angle, +max_angle].
    """
    rng = np.random.default_rng(seed)
    return rng.uniform(-max_angle_deg, max_angle_deg, n_segments)


# ---------------------------------------------------------------------------
# Registry — maps string name to function for CLI / Monte Carlo use
# ---------------------------------------------------------------------------

PROFILES = {
    "flat":        flat,
    "incline":     incline,
    "decline":     decline,
    "flat_top":    flat_top,
    "flat_bottom": flat_bottom,
    "bumpy":       bumpy,
}


def get_profile(name: str, n_segments: int, **kwargs) -> np.ndarray:
    """
    Retrieve a terrain profile array by name.

    Parameters
    ----------
    name : str
        One of: 'flat', 'incline', 'decline', 'flat_top', 'flat_bottom', 'bumpy'
    n_segments : int
        Number of road segments.
    **kwargs
        Additional keyword arguments passed to the profile function
        (e.g. angle_deg, seed).

    Returns
    -------
    np.ndarray
        Gradient array in degrees.

    Raises
    ------
    ValueError
        If the profile name is not recognised.
    """
    if name not in PROFILES:
        raise ValueError(f"Unknown terrain profile '{name}'. "
                         f"Available: {list(PROFILES.keys())}")
    return PROFILES[name](n_segments, **kwargs)


# ---------------------------------------------------------------------------
# Quick visualisation
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    n = 8
    fig, axes = plt.subplots(3, 2, figsize=(10, 8))
    fig.suptitle("Terrain Profiles — 8 Segments", fontsize=13)

    for ax, (name, fn) in zip(axes.flat, PROFILES.items()):
        profile = fn(n, seed=42) if name == "bumpy" else fn(n)
        ax.bar(range(1, n + 1), profile, color="steelblue", edgecolor="k", linewidth=0.5)
        ax.axhline(0, color="k", linewidth=0.7)
        ax.set_title(name.replace("_", " ").title())
        ax.set_xlabel("Segment")
        ax.set_ylabel("Gradient (°)")
        ax.set_ylim(-4, 4)
        ax.grid(axis="y", alpha=0.4)

    plt.tight_layout()
    plt.savefig("results/terrain_profiles_overview.png", dpi=150, bbox_inches="tight")
    plt.show()
