"""Fixed representation tolerances, independent of user-selected physics tolerances."""

import numpy as np

# A pair of float32 round trips can perturb normalized orientations as well as
# individual coordinates. Keep this allowance small and report it explicitly.
FRAME_RTOL = 2 * float(np.finfo(np.float32).eps)
FRAME_POSITION_ATOL = 1e-7  # metres; float32 transform composition can cancel small coordinates
FRAME_ROTATION_ATOL = FRAME_RTOL  # dimensionless matrix entries


def positions_match(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return bool(
        np.all(np.abs(a - b) <= FRAME_POSITION_ATOL + FRAME_RTOL * np.maximum(np.abs(a), np.abs(b)))
    )


def rotations_match(a, b):
    return bool(np.allclose(a, b, rtol=0, atol=FRAME_ROTATION_ATOL))


def poses_match(a, b):
    return positions_match(a[:3], b[:3]) and rotations_match(a[3:], b[3:])


def tensors_match(a, b, *, rtol, atol):
    """Compare physical tensors using a basis-invariant relative error.

    Elementwise relative tolerances overstate harmless errors on tiny/zero
    off-diagonal components created by rotating principal inertias.
    """
    a, b = np.asarray(a), np.asarray(b)
    return bool(np.linalg.norm(a - b) <= atol + rtol * max(np.linalg.norm(a), np.linalg.norm(b)))
