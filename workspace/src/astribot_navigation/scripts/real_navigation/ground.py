"""Estimate the nearby supporting plane in chassis coordinates, independently per cloud."""
import numpy as np


def ground_plane(base, nominal):
    # Equal area votes keep a dense object or one wall from dominating.
    near = (np.abs(base[:, 0]) < 3.5) & (np.abs(base[:, 1]) < 3.5)
    near &= np.abs(base[:, 2] - nominal) < .085
    points = base[near]
    if len(points) < 16:
        return None
    cells = np.floor(points[:, :2] / .35).astype(int) + 10
    ids = cells[:, 0] * 20 + cells[:, 1]
    order = np.lexsort((points[:, 2], ids))
    _, starts, counts = np.unique(ids[order], return_index=True, return_counts=True)
    indices = order[starts[counts >= 2] + counts[counts >= 2] // 4]
    if len(indices) < 10:
        return None
    samples = points[indices]
    xy = samples[:, :2]
    design = np.column_stack((xy, np.ones(len(xy))))
    # Start near the measured chassis clearance. Select a lower supported
    # surface and reject unsupported, steep or one-sided patches.
    rng = np.random.default_rng(1701)
    picks = rng.integers(len(xy), size=(96, 3))
    triples = design[picks]
    valid = np.abs(np.linalg.det(triples)) > .25
    if not valid.any():
        return None
    planes = np.linalg.solve(triples[valid], samples[picks[valid], 2, None])[:, :, 0]
    planes = np.vstack((planes, [0., 0., nominal]))
    planes = planes[(np.linalg.norm(planes[:, :2], axis=1) < .065) &
                    (np.abs(planes[:, 2] - nominal) < .018)]
    residual = samples[:, 2, None] - design @ planes.T
    support = np.abs(residual) < .011
    score = support.sum(axis=0) - 2 * (residual < -.016).sum(axis=0)
    score = score - len(xy) * (3 * np.abs(planes[:, 2] - nominal) + 2 * np.linalg.norm(planes[:, :2], axis=1))
    best = np.argmax(score)
    good = support[:, best]
    if good.sum() < max(10, .35 * len(xy)):
        return None
    angles = np.arctan2(xy[good, 1], xy[good, 0])
    gaps = np.diff(np.r_[np.sort(angles), angles.min() + 2 * np.pi])
    if gaps.max() > 1.5 * np.pi or np.linalg.eigvalsh(np.cov(xy[good].T))[0] < .20:
        return None
    plane = np.linalg.lstsq(design[good], samples[good, 2], rcond=None)[0]
    if abs(plane[2] - nominal) > .018 or np.linalg.norm(plane[:2]) > .065:
        return None
    return plane


def height_above_ground(base, nominal):
    plane = ground_plane(base, nominal)
    if plane is None:
        # Sparse frames still use the chassis clearance. Never infer free space
        # from a wall, a small elevated object, or an unobserved patch.
        return base[:, 2] - (nominal - .012)
    # Limit extrapolation of slope beyond the observed local footprint.
    xy = np.clip(base[:, :2], -3.5, 3.5)
    return base[:, 2] - (xy @ plane[:2] + plane[2])
