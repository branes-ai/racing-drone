# apps/ — the harness executables, generated per baseline (cmake/baselines.cmake).
#
# Each baseline in baselines/ yields flight_stack_<name>, replay_<name>, and
# sitl_<name>, built from the sources in apps/<harness>/ against that
# baseline's generated composition header.

racing_add_baseline(null)
