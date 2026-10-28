# Manifests that violate the boundary, for boundary.detects_violation: the
# racing gate codec, listed as neutral, includes the racing perception header,
# and a fixture includes it through `<./branes/...>`.
set(RACING_NEUTRAL_FILES
    sdk/include/branes/racing/msg/gate_codec.hpp
    tests/tools/boundary_fixture/dot_include.hpp
)
