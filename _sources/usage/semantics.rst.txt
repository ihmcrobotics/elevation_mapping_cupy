.. _semantics:

Semantic and image fusion
=========================

The IHMC ``ihmc-jazzy`` branch supports semantic PointCloud2 fields and
calibrated image fusion onto valid elevation cells. Images alone do not
create elevation geometry.

Configure ``channels`` for semantic cloud fields or score-image bands.
Image subscriptions also require a matching CameraInfo topic and TF from
the optical frame to the map frame. RGB uses ``channels: [rgb]`` with the
``color`` image fusion rule; semantic scores use ``exponential``.

See ``IHMC_JAZZY.md`` at the repository root for runnable configuration,
Docker commands, source provenance and validation details.
