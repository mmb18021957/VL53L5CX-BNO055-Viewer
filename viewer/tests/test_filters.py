import numpy as np


class TemporalFilter:
    """Exponential moving average filter for temporal smoothing."""

    def __init__(self):
        self.prev = None

    def reset(self):
        self.prev = None

    def apply(self, distances, strength=0.5):
        if self.prev is None:
            self.prev = distances.astype(np.float32)
            return self.prev

        if strength <= 0.0:
            self.prev = distances.astype(np.float32)
            return self.prev

        if strength >= 1.0:
            return self.prev

        filtered = self.prev * strength + distances * (1.0 - strength)
        self.prev = filtered.astype(np.float32)
        return self.prev


def fit_plane(points):
    if points.shape[0] < 3:
        return None

    centroid = np.mean(points, axis=0)
    centered = points - centroid

    try:
        _, _, vh = np.linalg.svd(centered)
    except np.linalg.LinAlgError:
        return None

    normal = vh[-1]
    if np.linalg.norm(normal) < 1e-6:
        return None

    normal = normal / np.linalg.norm(normal)

    z = np.array([0, 0, 1], dtype=np.float32)
    dot = np.dot(z, normal)

    if abs(dot - 1.0) < 1e-6:
        quat = np.array([1, 0, 0, 0], dtype=np.float32)
    elif abs(dot + 1.0) < 1e-6:
        quat = np.array([0, 1, 0, 0], dtype=np.float32)
    else:
        axis = np.cross(z, normal)
        axis /= np.linalg.norm(axis)
        angle = np.arccos(dot)
        w = np.cos(angle / 2)
        xyz = axis * np.sin(angle / 2)
        quat = np.array([w, xyz[0], xyz[1], xyz[2]], dtype=np.float32)

    size = np.ptp(points, axis=0)

    return centroid.astype(np.float32), quat.astype(np.float32), size.astype(np.float32)


def fit_plane_ransac(points, threshold=0.05, iterations=100):
    n = points.shape[0]
    if n < 3:
        return None

    best_inliers = []
    best_model = None

    for _ in range(iterations):
        idx = np.random.choice(n, 3, replace=False)
        sample = points[idx]

        model = fit_plane(sample)
        if model is None:
            continue

        pos, quat, _ = model

        w, x, y, z = quat
        normal = np.array([
            2 * (x * z + w * y),
            2 * (y * z - w * x),
            1 - 2 * (x * x + y * y)
        ])

        diffs = points - pos
        dists = np.abs(np.dot(diffs, normal))

        inliers = np.where(dists < threshold)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = model

    if best_model is None or len(best_inliers) < 3:
        return None

    return fit_plane(points[best_inliers])
