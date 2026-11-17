"""Move A then B to the Viser Marvin Pro rest poses. Read-only unless --execute."""

import math

from home_both_arms import main

# Viser robots/marvin_pro/spec.yaml lists rest_q in radians.
# This station uses SDK channel A for physical right and B for physical left.
REST_RAD = {
    "A": [-1.2, -1.2, 1.2, -1.6, 0.0, 0.0, 0.0],
    "B": [1.2, -1.2, -1.2, -1.6, 0.0, 0.0, 0.0],
}


if __name__ == "__main__":
    targets = {channel: [math.degrees(q) for q in pose] for channel, pose in REST_RAD.items()}
    main(targets, description=__doc__)
