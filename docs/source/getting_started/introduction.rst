.. _introduction:

Introduction
******************************************************************

Elevation Mapping CuPy is a ROS 2 (Jazzy) node that builds and maintains an elevation map on the GPU
using CuPy kernels.

This Jazzy branch intentionally keeps the supported feature set small and deterministic:

* PointCloud2 geometry with optional semantic fields and calibrated image fusion.
* One regression-tested bring-up path:

  * synthetic TF (``map -> base_link``) + synthetic depth-like pointcloud
  * ``elevation_mapping_node.py`` publishes a ``grid_map_msgs/msg/GridMap`` on
    ``/elevation_mapping_node/elevation_map``

The IHMC ``ihmc-jazzy`` branch restores multi-modal fusion. See ``IHMC_JAZZY.md`` for setup and validation.

