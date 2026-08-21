#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/perception/depth_image_to_point_cloud.h"
// #include "drake/perception/point_cloud.h"
// #include "drake/perception/point_cloud_flags.h"
// #include "drake/perception/point_cloud_to_lcm.h"

// Symbol: pydrake_doc_perception
constexpr struct /* pydrake_doc_perception */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::perception
    struct /* perception */ {
      // Symbol: drake::perception::Concatenate
      struct /* Concatenate */ {
        // Source: drake/perception/point_cloud.h
        const char* doc =
R"""(Returns a new point cloud that includes all of the points from the
point clouds in ``clouds``. All of the ``clouds`` must have the same
fields.

Precondition:
    ``clouds`` contains at least one point cloud.

Raises:
    RuntimeError if the clouds have different fields defined.)""";
      } Concatenate;
      // Symbol: drake::perception::DepthImageToPointCloud
      struct /* DepthImageToPointCloud */ {
        // Source: drake/perception/depth_image_to_point_cloud.h
        const char* doc =
R"""(Converts a depth image to a point cloud.

.. pydrake_system::

    name: DepthImageToPointCloud
    input_ports:
    - depth_image
    - color_image (optional)
    - camera_pose (optional)
    output_ports:
    - point_cloud

The system has an input port that takes a depth image, an optional
input port that takes a color image, and an additional optional input
port that takes the camera_pose, X_PC. If the camera_pose input is
connected, then the point cloud is represented in the parent frame
(e.g., if camera_pose is the pose of the camera in the world frame,
then the point_cloud output will be a PointCloud in the world frame).
If the camera_pose input is not connected, the PointCloud will be
represented in the camera frame. Note that if a color image is
provided, it must be in the same frame as the depth image.

If a pixel is NaN, the converted point will be (NaN, NaN, NaN). If a
pixel is kTooClose or kTooFar (as defined by ImageTraits), the
converted point will be (+Inf, +Inf, +Inf). Note that this matches the
convention used by the Point Cloud Library (PCL).)""";
        // Symbol: drake::perception::DepthImageToPointCloud::Convert
        struct /* Convert */ {
          // Source: drake/perception/depth_image_to_point_cloud.h
          const char* doc =
R"""(Converts a depth image to a point cloud using direct arguments instead
of System input and output ports. The semantics are the same as
documented in the class overview and constructor.

Parameter ``cloud``:
    Destination for point data; must not be nullptr. The ``cloud``
    will be resized to match the size of the depth image. The
    ``cloud`` must have the XYZ channel enabled.)""";
        } Convert;
        // Symbol: drake::perception::DepthImageToPointCloud::DepthImageToPointCloud
        struct /* ctor */ {
          // Source: drake/perception/depth_image_to_point_cloud.h
          const char* doc =
R"""(Constructs the converter.

Parameter ``camera_info``:
    The camera info.

Parameter ``depth_pixel_type``:
    The pixel type of the depth image input. Only 16U and 32F are
    supported.

Parameter ``scale``:
    The depth image input is multiplied by this scale factor before
    projecting to a point cloud. (This is useful for converting mm to
    meters, etc.)

Parameter ``fields``:
    The fields the point cloud contains.)""";
        } ctor;
        // Symbol: drake::perception::DepthImageToPointCloud::camera_pose_input_port
        struct /* camera_pose_input_port */ {
          // Source: drake/perception/depth_image_to_point_cloud.h
          const char* doc =
R"""(Returns the abstract valued input port that expects X_PC as a
RigidTransformd. (This input port does not necessarily need to be
connected; refer to the class overview for details.))""";
        } camera_pose_input_port;
        // Symbol: drake::perception::DepthImageToPointCloud::color_image_input_port
        struct /* color_image_input_port */ {
          // Source: drake/perception/depth_image_to_point_cloud.h
          const char* doc =
R"""(Returns the abstract valued input port that expects an ImageRgba8U.)""";
        } color_image_input_port;
        // Symbol: drake::perception::DepthImageToPointCloud::depth_image_input_port
        struct /* depth_image_input_port */ {
          // Source: drake/perception/depth_image_to_point_cloud.h
          const char* doc =
R"""(Returns the abstract valued input port that expects either an
ImageDepth16U or ImageDepth32F (depending on the constructor
argument).)""";
        } depth_image_input_port;
        // Symbol: drake::perception::DepthImageToPointCloud::point_cloud_output_port
        struct /* point_cloud_output_port */ {
          // Source: drake/perception/depth_image_to_point_cloud.h
          const char* doc =
R"""(Returns the abstract valued output port that provides a PointCloud.
Only the channels passed into the constructor argument "fields" are
present.)""";
        } point_cloud_output_port;
      } DepthImageToPointCloud;
      // Symbol: drake::perception::PointCloud
      struct /* PointCloud */ {
        // Source: drake/perception/point_cloud.h
        const char* doc =
R"""(Implements a point cloud (with contiguous storage), whose main goal is
to offer a convenient, synchronized interface to commonly used fields
and data types applicable for basic 3D perception.

This is a mix between the philosophy of PCL (templated interface to
provide a compile-time open set, run-time closed set) and VTK
(non-templated interface to provide a very free form run-time open
set).

Definitions:

- point - An entry in a point cloud (not exclusively an XYZ point).
- feature - Abstract representation of local properties (geometric and
  non-geometric)
- descriptor - Concrete representation of a feature.
- field - A feature or descriptor described by the point cloud.

This point cloud class provides the following fields:

- xyz - Cartesian XYZ coordinates (float[3]).
- descriptor - A descriptor that is run-time defined (float[X]).

Note:
    "contiguous" here means contiguous in memory. This was chosen to
    avoid ambiguity between PCL and Eigen, where in PCL "dense"
    implies that the point cloud corresponds to a cloud with only
    valid values, and in Eigen "dense" implies contiguous storage.

Note:
    The accessors / mutators for the point fields of this class
    returns references to the original Eigen matrices. This implies
    that they are invalidated whenever memory is reallocated for the
    values. Given this, minimize the lifetime of these references to
    be as short as possible. Additionally, algorithms wanting fast
    access to values should avoid the single point accessors /
    mutatotrs (e.g. ``xyz(i)``, `mutable_descriptor(i)`) to avoid
    overhead when accessing a single element (either copying or
    creating a reference).

Note:
    The definitions presented here for "feature" and "descriptor" are
    loosely based on their definitions within PCL and Radu Rusu's
    dissertation: Rusu, Radu Bogdan. "Semantic 3d object maps for
    everyday manipulation in human living environments." KI-Künstliche
    Intelligenz 24.4 (2010): 345-348. This differs from other
    definitions, such as having "feature" describe geometric
    quantities and "descriptor" describe non-geometric quantities
    which is presented in the following survey paper: Pomerleau,
    François, Francis Colas, and Roland Siegwart. "A review of point
    cloud registration algorithms for mobile robotics." Foundations
    and Trends® in Robotics 4.1 (2015): 1-104.)""";
        // Symbol: drake::perception::PointCloud::C
        struct /* C */ {
          // Source: drake/perception/point_cloud.h
          const char* doc = R"""(Color channel scalar type.)""";
        } C;
        // Symbol: drake::perception::PointCloud::Crop
        struct /* Crop */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns a new point cloud containing only the points in ``this`` with
xyz values within the axis-aligned bounding box defined by
``lower_xyz`` and ``upper_xyz``. Requires that xyz values are defined.

Precondition:
    lower_xyz <= upper_xyz (elementwise).

Raises:
    RuntimeError if has_xyzs() != true.)""";
        } Crop;
        // Symbol: drake::perception::PointCloud::D
        struct /* D */ {
          // Source: drake/perception/point_cloud.h
          const char* doc = R"""(Descriptor scalar type.)""";
        } D;
        // Symbol: drake::perception::PointCloud::EstimateNormals
        struct /* EstimateNormals */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Estimates the normal vectors in ``this`` by fitting a plane at each
point in the cloud using up to ``num_closest`` points within Euclidean
distance ``radius`` from the point. If has_normals() is false, then
new normals will be allocated (and has_normals() will become true).
Points for which the normals cannot be estimated (because the ``this``
has less than two closest points within the ``radius)``, will receive
normal [NaN, NaN, NaN]. Normals estimated from two closest points will
be orthogonal to the vector between those points, but can be arbitrary
in the last dimension. ``parallelize`` enables OpenMP parallelization.

Returns:
    true iff all points were assigned normals by having at least
    *three* closest points within ``radius``.

Precondition:
    ``radius`` > 0 and ``num_closest`` >= 3.

Raises:
    RuntimeError if has_xyzs() is false.)""";
        } EstimateNormals;
        // Symbol: drake::perception::PointCloud::Expand
        struct /* Expand */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Adds ``add_size`` default-initialized points.

Parameter ``add_size``:
    Number of points to add.

Parameter ``skip_initialization``:
    Do not require that the new values be initialized.)""";
        } Expand;
        // Symbol: drake::perception::PointCloud::FlipNormalsTowardPoint
        struct /* FlipNormalsTowardPoint */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Changes the sign of the normals in ``this``, if necessary, so that
each normal points toward the point ``P`` in the frame ``C`` in which
the xyzs of ``this`` cloud are represented. This can be useful, for
instance, when ``P`` is the position of the camera used to generate
the cloud.

Raises:
    RuntimeError if has_xyzs() != true or has_normals() != true.)""";
        } FlipNormalsTowardPoint;
        // Symbol: drake::perception::PointCloud::HasExactFields
        struct /* HasExactFields */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns if a point cloud has exactly a given set of fields.

See also:
    HasFields for preconditions.)""";
        } HasExactFields;
        // Symbol: drake::perception::PointCloud::HasFields
        struct /* HasFields */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns if a point cloud has a given set of fields.)""";
        } HasFields;
        // Symbol: drake::perception::PointCloud::IsDefaultValue
        struct /* IsDefaultValue */ {
          // Source: drake/perception/point_cloud.h
          const char* doc = R"""()""";
        } IsDefaultValue;
        // Symbol: drake::perception::PointCloud::IsInvalidValue
        struct /* IsInvalidValue */ {
          // Source: drake/perception/point_cloud.h
          const char* doc = R"""()""";
        } IsInvalidValue;
        // Symbol: drake::perception::PointCloud::PointCloud
        struct /* ctor */ {
          // Source: drake/perception/point_cloud.h
          const char* doc_3args =
R"""(Constructs a point cloud of a given ``new_size``, with the prescribed
``fields``. If ``kDescriptors`` is one of the fields, then
``descriptor`` should be included and should not be ``kNone``.

Parameter ``new_size``:
    Size of the point cloud after construction.

Parameter ``fields``:
    Fields that the point cloud contains.

Parameter ``skip_initialize``:
    Do not default-initialize new values.)""";
          // Source: drake/perception/point_cloud.h
          const char* doc_copy =
R"""(Copies another point cloud's fields and data.)""";
          // Source: drake/perception/point_cloud.h
          const char* doc_move =
R"""(Takes ownership of another point cloud's data.)""";
          // Source: drake/perception/point_cloud.h
          const char* doc_2args =
R"""(Copies another point cloud's fields and data.

Parameter ``copy_fields``:
    Fields to copy. If this is ``kInherit``, then `other`s fields will
    be copied. Otherwise, only the specified fields will be copied;
    the remaining fields in this cloud are left default initialized.)""";
        } ctor;
        // Symbol: drake::perception::PointCloud::RequireExactFields
        struct /* RequireExactFields */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Requires the exact given set of fields.

See also:
    HasFields for preconditions.

Raises:
    RuntimeError if this point cloud does not have exactly these
    fields.)""";
        } RequireExactFields;
        // Symbol: drake::perception::PointCloud::RequireFields
        struct /* RequireFields */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Requires a given set of fields.

See also:
    HasFields for preconditions.

Raises:
    RuntimeError if this point cloud does not have these fields.)""";
        } RequireFields;
        // Symbol: drake::perception::PointCloud::SetFields
        struct /* SetFields */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Updates the point cloud to a given set of fields. In the case of
introducing a new field, its container will be allocated with the
current size and default initialized. The data for all retained fields
will remain unchanged.

Parameter ``new_fields``:
    New fields to set to.

Parameter ``skip_initialize``:
    Do not default-initialize new values.)""";
        } SetFields;
        // Symbol: drake::perception::PointCloud::SetFrom
        struct /* SetFrom */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Copies all points from another point cloud.

Parameter ``other``:
    Other point cloud.

Parameter ``fields_in``:
    Fields to copy. If this is ``kInherit``, then `other`s fields will
    be copied. Otherwise, both clouds must support the fields
    indicated this parameter.

Parameter ``allow_resize``:
    Permit resizing to the other cloud's size.)""";
        } SetFrom;
        // Symbol: drake::perception::PointCloud::T
        struct /* T */ {
          // Source: drake/perception/point_cloud.h
          const char* doc = R"""(Geometric scalar type.)""";
        } T;
        // Symbol: drake::perception::PointCloud::VoxelizedDownSample
        struct /* VoxelizedDownSample */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns a down-sampled point cloud by grouping all xyzs in this cloud
into a 3D grid with cells of dimension voxel_size. Each occupied voxel
will result in one point in the downsampled cloud, with a location
corresponding to the centroid of the points in that voxel. Points with
non-finite xyz values are ignored. All other fields (e.g. rgbs,
normals, and descriptors) with finite values will also be averaged
across the points in a voxel. ``parallelize`` enables OpenMP
parallelization. Equivalent to Open3d's voxel_down_sample or PCL's
VoxelGrid filter.

Raises:
    RuntimeError if has_xyzs() is false.

Raises:
    RuntimeError if voxel_size <= 0.)""";
        } VoxelizedDownSample;
        // Symbol: drake::perception::PointCloud::descriptor
        struct /* descriptor */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to a descriptor value.

Precondition:
    ``has_descriptors()`` must be true.)""";
        } descriptor;
        // Symbol: drake::perception::PointCloud::descriptor_type
        struct /* descriptor_type */ {
          // Source: drake/perception/point_cloud.h
          const char* doc = R"""(Returns the descriptor type.)""";
        } descriptor_type;
        // Symbol: drake::perception::PointCloud::descriptors
        struct /* descriptors */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to descriptor values.

Precondition:
    ``has_descriptors()`` must be true.)""";
        } descriptors;
        // Symbol: drake::perception::PointCloud::fields
        struct /* fields */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns the fields provided by this point cloud.)""";
        } fields;
        // Symbol: drake::perception::PointCloud::has_descriptors
        struct /* has_descriptors */ {
          // Source: drake/perception/point_cloud.h
          const char* doc_0args =
R"""(Returns if this point cloud provides descriptor values.)""";
          // Source: drake/perception/point_cloud.h
          const char* doc_1args =
R"""(Returns if the point cloud provides a specific descriptor.)""";
        } has_descriptors;
        // Symbol: drake::perception::PointCloud::has_normals
        struct /* has_normals */ {
          // Source: drake/perception/point_cloud.h
          const char* doc = R"""(Returns if this cloud provides normals.)""";
        } has_normals;
        // Symbol: drake::perception::PointCloud::has_rgbs
        struct /* has_rgbs */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns if this cloud provides RGB colors.)""";
        } has_rgbs;
        // Symbol: drake::perception::PointCloud::has_xyzs
        struct /* has_xyzs */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns if this cloud provides XYZ values.)""";
        } has_xyzs;
        // Symbol: drake::perception::PointCloud::mutable_descriptor
        struct /* mutable_descriptor */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to a descriptor value.

Precondition:
    ``has_descriptors()`` must be true.)""";
        } mutable_descriptor;
        // Symbol: drake::perception::PointCloud::mutable_descriptors
        struct /* mutable_descriptors */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to descriptor values.

Precondition:
    ``has_descriptors()`` must be true.)""";
        } mutable_descriptors;
        // Symbol: drake::perception::PointCloud::mutable_normal
        struct /* mutable_normal */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to a normal.

Precondition:
    ``has_normals()`` must be true.)""";
        } mutable_normal;
        // Symbol: drake::perception::PointCloud::mutable_normals
        struct /* mutable_normals */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to normals.

Precondition:
    ``has_normals()`` must be true.)""";
        } mutable_normals;
        // Symbol: drake::perception::PointCloud::mutable_rgb
        struct /* mutable_rgb */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to an RGB color.

Precondition:
    ``has_rgbs()`` must be true.)""";
        } mutable_rgb;
        // Symbol: drake::perception::PointCloud::mutable_rgbs
        struct /* mutable_rgbs */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to RGB colors.

Precondition:
    ``has_rgbs()`` must be true.)""";
        } mutable_rgbs;
        // Symbol: drake::perception::PointCloud::mutable_xyz
        struct /* mutable_xyz */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to an XYZ value.

Precondition:
    ``has_xyzs()`` must be true.)""";
        } mutable_xyz;
        // Symbol: drake::perception::PointCloud::mutable_xyzs
        struct /* mutable_xyzs */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns mutable access to XYZ values.

Precondition:
    ``has_xyzs()`` must be true.)""";
        } mutable_xyzs;
        // Symbol: drake::perception::PointCloud::normal
        struct /* normal */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to a normal.

Precondition:
    ``has_normals()`` must be true.)""";
        } normal;
        // Symbol: drake::perception::PointCloud::normals
        struct /* normals */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to normals.

Precondition:
    ``has_normals()`` must be true.)""";
        } normals;
        // Symbol: drake::perception::PointCloud::resize
        struct /* resize */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Conservative resize; will maintain existing data, and initialize new
data to their invalid values.

Parameter ``new_size``:
    The new size of the value. If less than the present ``size()``,
    then the values will be truncated. If greater than the present
    ``size()``, then the new values will be uninitialized if
    ``skip_initialize`` is not true.

Parameter ``skip_initialize``:
    Do not default-initialize new values.)""";
        } resize;
        // Symbol: drake::perception::PointCloud::rgb
        struct /* rgb */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to an RGB color.

Precondition:
    ``has_rgbs()`` must be true.)""";
        } rgb;
        // Symbol: drake::perception::PointCloud::rgbs
        struct /* rgbs */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to RGB colors.

Precondition:
    ``has_rgbs()`` must be true.)""";
        } rgbs;
        // Symbol: drake::perception::PointCloud::size
        struct /* size */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns the number of points in this point cloud.)""";
        } size;
        // Symbol: drake::perception::PointCloud::xyz
        struct /* xyz */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to an XYZ value.

Precondition:
    ``has_xyzs()`` must be true.)""";
        } xyz;
        // Symbol: drake::perception::PointCloud::xyzs
        struct /* xyzs */ {
          // Source: drake/perception/point_cloud.h
          const char* doc =
R"""(Returns access to XYZ values.

Precondition:
    ``has_xyzs()`` must be true.)""";
        } xyzs;
      } PointCloud;
      // Symbol: drake::perception::PointCloudToLcm
      struct /* PointCloudToLcm */ {
        // Source: drake/perception/point_cloud_to_lcm.h
        const char* doc =
R"""(Converts PointCloud inputs to lcmt_point_cloud output messages. The
message can be transmitted to other processes using
LcmPublisherSystem.

.. pydrake_system::

    name: PointCloudToLcm
    input_ports:
    - point_cloud
    output_ports:
    - lcmt_point_cloud

Any descriptor channels of the PointCloud will currently be ignored,
though may be added in a future revision.

Only the finite points from the cloud are copied into the message
(too-close or too-far points from a depth sensor are omitted).)""";
        // Symbol: drake::perception::PointCloudToLcm::PointCloudToLcm
        struct /* ctor */ {
          // Source: drake/perception/point_cloud_to_lcm.h
          const char* doc =
R"""(Constructs a system that outputs messages using the given
``frame_name``.)""";
        } ctor;
      } PointCloudToLcm;
      // Symbol: drake::perception::pc_flags
      struct /* pc_flags */ {
        // Symbol: drake::perception::pc_flags::BaseField
        struct /* BaseField */ {
          // Source: drake/perception/point_cloud_flags.h
          const char* doc =
R"""(Indicates the data the point cloud stores.)""";
          // Symbol: drake::perception::pc_flags::BaseField::kInherit
          struct /* kInherit */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Inherit other fields. May imply an intersection of all compatible
descriptors.)""";
          } kInherit;
          // Symbol: drake::perception::pc_flags::BaseField::kNone
          struct /* kNone */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""()""";
          } kNone;
          // Symbol: drake::perception::pc_flags::BaseField::kNormals
          struct /* kNormals */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""(Normals.)""";
          } kNormals;
          // Symbol: drake::perception::pc_flags::BaseField::kRGBs
          struct /* kRGBs */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""(RGB colors.)""";
          } kRGBs;
          // Symbol: drake::perception::pc_flags::BaseField::kXYZs
          struct /* kXYZs */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""(XYZ point in Cartesian space.)""";
          } kXYZs;
        } BaseField;
        // Symbol: drake::perception::pc_flags::BaseFieldT
        struct /* BaseFieldT */ {
          // Source: drake/perception/point_cloud_flags.h
          const char* doc = R"""()""";
        } BaseFieldT;
        // Symbol: drake::perception::pc_flags::DescriptorType
        struct /* DescriptorType */ {
          // Source: drake/perception/point_cloud_flags.h
          const char* doc =
R"""(Describes an descriptor field with a name and the descriptor's size.

Note:
    This is defined as follows to enable an open set of descriptors,
    but ensure that these descriptor types are appropriately matched.
    As ``PointCloud`` evolves and more algorithms are mapped into
    Drake, promoting an descriptor field to a proper field should be
    considered if (a) it is used frequently enough AND (b) if it is
    often used in conjunction with other fields.)""";
          // Symbol: drake::perception::pc_flags::DescriptorType::DescriptorType
          struct /* ctor */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::perception::pc_flags::DescriptorType::name
          struct /* name */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""()""";
          } name;
          // Symbol: drake::perception::pc_flags::DescriptorType::operator!=
          struct /* operator_ne */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""()""";
          } operator_ne;
          // Symbol: drake::perception::pc_flags::DescriptorType::size
          struct /* size */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""()""";
          } size;
        } DescriptorType;
        // Symbol: drake::perception::pc_flags::Fields
        struct /* Fields */ {
          // Source: drake/perception/point_cloud_flags.h
          const char* doc =
R"""(Allows combination of ``BaseField`` and ``DescriptorType`` for a
``PointCloud``. You may combine multiple ``BaseField`s, but you may
have only zero or one `DescriptorType``.

This provides the mechanism to use basic bit-mask operators (| &) to
combine / intersect fields for convenience.)""";
          // Symbol: drake::perception::pc_flags::Fields::Fields
          struct /* ctor */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Raises:
    RuntimeError if ``base_fields`` is not composed of valid
    `BaseField`s.)""";
          } ctor;
          // Symbol: drake::perception::pc_flags::Fields::base_fields
          struct /* base_fields */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""(Returns the contained base fields.)""";
          } base_fields;
          // Symbol: drake::perception::pc_flags::Fields::contains
          struct /* contains */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Returns whether this set of fields contains (is a superset of)
``rhs``.)""";
          } contains;
          // Symbol: drake::perception::pc_flags::Fields::descriptor_type
          struct /* descriptor_type */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""(Returns the contained descriptor type.)""";
          } descriptor_type;
          // Symbol: drake::perception::pc_flags::Fields::empty
          struct /* empty */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Returns whether both value types (BaseField + DescriptorType) are
none.)""";
          } empty;
          // Symbol: drake::perception::pc_flags::Fields::has_base_fields
          struct /* has_base_fields */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Returns whether there are any base fields contained by this set of
fields.)""";
          } has_base_fields;
          // Symbol: drake::perception::pc_flags::Fields::has_descriptor
          struct /* has_descriptor */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Returns whether there is a descriptor contained by this set of fields.)""";
          } has_descriptor;
          // Symbol: drake::perception::pc_flags::Fields::operator!=
          struct /* operator_ne */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""()""";
          } operator_ne;
          // Symbol: drake::perception::pc_flags::Fields::operator&
          struct /* operator_band */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""(Provides intersection.)""";
          } operator_band;
          // Symbol: drake::perception::pc_flags::Fields::operator&=
          struct /* operator_iand */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc = R"""(Provides in-place intersection.)""";
          } operator_iand;
          // Symbol: drake::perception::pc_flags::Fields::operator|
          struct /* operator_bor */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Provides union.

See also:
    operator|= for preconditions.)""";
          } operator_bor;
          // Symbol: drake::perception::pc_flags::Fields::operator|=
          struct /* operator_ior */ {
            // Source: drake/perception/point_cloud_flags.h
            const char* doc =
R"""(Provides in-place union.

Raises:
    RuntimeError if multiple non-None `DescriptorType`s are specified.)""";
          } operator_ior;
        } Fields;
        // Symbol: drake::perception::pc_flags::operator|
        struct /* operator_bor */ {
          // Source: drake/perception/point_cloud_flags.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Makes operator| compatible for ``BaseField`` + ``DescriptorType``.

See also:
    Fields∷operator|= for preconditions.)""";
        } operator_bor;
        // Symbol: drake::perception::pc_flags::to_string
        struct /* to_string */ {
          // Source: drake/perception/point_cloud_flags.h
          const char* doc =
R"""(Provides a human-readable description of ``fields``.)""";
        } to_string;
      } pc_flags;
    } perception;
  } drake;
} pydrake_doc_perception;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
