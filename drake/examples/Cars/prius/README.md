# Introduction

This folder contains several models of the Prius sedan in both URDF and SDF format. The URDF and SDF files should be kimatically and dynamically equivalent with a tolerance smaller than the system's epsilon.

There is an issue where the order of elements inside of the SDF matter. To verify this, prius_reordered.sdf includes XML elements that are in a different order relative to prius.sdf but are otherwise the same. Loading prius_reordered.sdf into Drake shows an obvious problem regarding the location of the front wheel.
