# Introduction

This folder contains multiple versions of the Prius vehicle in two different formats.

prius_v0.sdf, prius_v0.urdf, and prius_v0_rear_wheel_drive.urdf are the original ones that were included with Drake. The URDF works in simulation but is a strange model where the axles are on the floor, a suspension system is modeled, and there is a "contact point" under each wheel. The SDF does not work in simulation and also does not match the URDF in terms of both kinematics and dynamics.

prius_v1.sdf and prius_v2.urdf are kimatically and dynamically equal and both work in simulation. There is an issue where the order of elements inside of the SDF matter. To verify this, prius_v1_reordered.sdf includes XML elements that are in a different order relative to prius_v1.sdf but are otherwise the same. Loading prius_v1_reordered.sdf into Drake shows an obvious problem regarding the location of the front wheel.
