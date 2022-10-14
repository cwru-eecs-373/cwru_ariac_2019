# Gazebo Environment for Agile Robotics (GEAR)

**This repository has been updated to work with ROS Noetic/Ubuntu Focal.  It *will* work fine once the fix for a small bug in the Python3 empy packaged is pushed (3.3.2-5.1->3.3.4).**

GEAR is the software used by teams participating in the Agile Robotics for
Industrial Automation Competition (ARIAC) hosted by the National Institute
of Standards and Technology (NIST).

Please see the [wiki](https://bitbucket.org/osrf/ariac/wiki) for installation instructions.


This repository contains the source code for GEAR.
Most participants will not need to build GEAR from source: please see the binary installation instructions instead.

The `master` branch is the "bleeding edge" development branch for the active competition year (2019).

To access the source code for the version of GEAR used in ARIAC 2018, use the [`ariac_2018` branch](https://bitbucket.org/osrf/ariac/src/812c23ff96c71baf7581ec412597de25333e4e02/?at=ariac_2018).
To access the source code for the version of GEAR used in ARIAC 2017, use the [`ariac_2017` branch](https://bitbucket.org/osrf/ariac/src/7342fec80e5612230710f82bf918f13b4dc4b08b/?at=ariac_2017).

---

We acknowledge the original authors of the `ur_description` and `ur_gazebo` packages, which we have modified to work with the ARIAC simulation and embedded in the `osrf_gear/vendor` directory.
