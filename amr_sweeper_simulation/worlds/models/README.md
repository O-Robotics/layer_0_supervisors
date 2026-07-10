# AMR Sweeper Simulation Models

This directory contains the Gazebo model assets used by the package-local
simulation worlds.

Most city environment models were copied from `osrf/gazebo_models` at commit
`8163eb4b5e7e21985c6591d1c0bfb56468c0093f` so the simulation can resolve the
model references used by the `small_city`, `test_city`, `neighborhood`, and
`simple_city` worlds without modifying the checked-out model/world collections.

The `city_terrain` and `ocean` models originate from `osrf/citysim` at commit
`3928b08e2598f5ead2e9b24640fe1bde262a136d`. The `asphalt_plane` model
originates from `leonhartyao/gazebo_models_worlds_collection` at commit
`cce115b82691b7c529a02f47e5efa391145a4ca1`, with Gazebo Sim material
compatibility fixes applied in this package-local copy.

These two repositories used to be vendored in full as git submodules under
`worlds/citysim` and `worlds/gazebo_models_worlds_collection`, but were removed
because only a handful of their hundreds of models are actually used by the
`small_city`, `test_city`, `neighborhood`, and `simple_city` worlds. Only the
models actually referenced by those four worlds are kept here. If a world
needs a model that isn't in this directory, pull it from the source repo/
commit below (or a newer commit, re-verifying compatibility) rather than
re-adding the repos as submodules:

- https://github.com/osrf/citysim (commit `3928b08e2598f5ead2e9b24640fe1bde262a136d`)
- https://github.com/leonhartyao/gazebo_models_worlds_collection (commit `cce115b82691b7c529a02f47e5efa391145a4ca1`)
- https://github.com/osrf/gazebo_models (commit `8163eb4b5e7e21985c6591d1c0bfb56468c0093f`)

The upstream `osrf/gazebo_models` repository is licensed under Creative Commons
Attribution 3.0 Unported. See:

- https://github.com/osrf/gazebo_models
- https://creativecommons.org/licenses/by/3.0/
