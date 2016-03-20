# Introduction

This folder contains models of Prius in two different formats: URDF and SDF. The files prius.sdf and prius.urdf are kimatically and dynamically equal and both work in simulation.

There is an issue where the order of elements inside of the SDF matter. To verify this, prius_reordered.sdf includes XML elements that are in a different order relative to prius.sdf but are otherwise the same. Loading prius_reordered.sdf into Drake shows an obvious problem regarding the location of the front wheel.
