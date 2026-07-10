# AMR Sweeper Simulation Models

This directory contains the Gazebo model assets used by the package-local
simulation worlds.

Most city environment models were copied from `osrf/gazebo_models` at commit
`8163eb4b5e7e21985c6591d1c0bfb56468c0093f` so the simulation can resolve the
model references used by the `small_city`, `test_city`, `neighborhood`, and
`simple_city` worlds without modifying the checked-out model/world collections.

The `actor`, `city_terrain`, and `ocean` models originate from the local
`worlds/citysim/models` collection. The `asphalt_plane` model originates from
the local `worlds/gazebo_models_worlds_collection/models` collection, with
Gazebo Sim material compatibility fixes applied in this package-local copy.

The upstream `osrf/gazebo_models` repository is licensed under Creative Commons
Attribution 3.0 Unported. See:

- https://github.com/osrf/gazebo_models
- https://creativecommons.org/licenses/by/3.0/
