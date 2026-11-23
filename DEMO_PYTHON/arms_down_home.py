"""Move A then B so both arms hang straight down. Read-only unless --execute."""

from home_both_arms import main

# Shoulder roll J1 turns the J2 swing into the vertical plane, then J2 drops the
# arm from the zero T-pose to vertical. J3 then spins each upper arm so the elbow
# bends forward instead of across the body (opposite signs, the arms are mirrored).
# Elbow J4 and the wrist stay at zero, so each arm is straight with the flange
# about 0.70 m below its shoulder.
# Derived from viser-teleop robots/marvin_pro. Vendor cabling since 2026-11-23:
# channel A is the left arm (port ET0 "LEFT"), channel B the right arm (ET2 "RIGHT").
# The sign of J1 depends on the arm; with the arms swapped these targets point up.
ARMS_DOWN_DEG = {
    "A": [90.0, -90.0, -90.0, 0.0, 0.0, 0.0, 0.0],
    "B": [-90.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0],
}


if __name__ == "__main__":
    main(ARMS_DOWN_DEG, description=__doc__)
