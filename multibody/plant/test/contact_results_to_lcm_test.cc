#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/never_destroyed.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::PenetrationAsPointPair;
using geometry::SceneGraph;
using geometry::Sphere;
using geometry::SurfaceTriangle;
using geometry::TriangleSurfaceMesh;
using math::RigidTransform;
using multibody::internal::FullBodyName;
using std::function;
using std::make_unique;
using std::move;
using std::nullopt;
using std::optional;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using symbolic::Expression;
using systems::Context;
using systems::DiagramBuilder;

namespace multibody {

/* Friend class to expose the internals of ContactResultsToLcmSystem. */
class ContactResultsToLcmTester {
 public:
  ContactResultsToLcmTester() = delete;

  /* Provide access to the hidden constructor that allows control over the
   naming method -- so we can directly test that it works. */
  template <typename T>
  static unique_ptr<ContactResultsToLcmSystem<T>> Make(
      const MultibodyPlant<T>& plant,
      const function<string(GeometryId)>& namer = nullptr) {
    /* We want to make sure we explicitly exercise both constructors. */
    if (namer == nullptr) {
      return unique_ptr<ContactResultsToLcmSystem<T>>(
          new ContactResultsToLcmSystem<T>(plant));
    } else {
      return unique_ptr<ContactResultsToLcmSystem<T>>(
          new ContactResultsToLcmSystem<T>(plant, namer));
    }
  }

  template <typename T>
  static unordered_map<GeometryId, FullBodyName>& get_geometry_id_to_body_map(
      ContactResultsToLcmSystem<T>* system) {
    return system->geometry_id_to_body_name_map_;
  }

  template <typename T>
  static void AddToGeometryBodyMap(
      ContactResultsToLcmSystem<T>* system,
      std::initializer_list<unordered_map<GeometryId, FullBodyName>::value_type>
          items) {
    system->geometry_id_to_body_name_map_.insert(items);
  }

  template <typename T>
  static vector<string>& get_body_names(ContactResultsToLcmSystem<T>* system) {
    return system->body_names_;
  }

  template <typename T>
  static void AddToBodyNames(ContactResultsToLcmSystem<T>* system,
                             std::initializer_list<string> items) {
    system->body_names_.insert(system->body_names_.end(), items);
  }

  template <typename T, typename U>
  static bool Equals(const ContactResultsToLcmSystem<T>& system_T,
                     const ContactResultsToLcmSystem<U>& system_U) {
    return system_T.Equals(system_U);
  }
};

/* Friend class to the plant. We're going to use this to artificially associate
 GeometryIds (for mythical collision geometries) with bodies. This removes the
 need for these tests to directly depend on SceneGraph. */
class MultibodyPlantTester {
 public:
  /* Adds the given geometry `id` to the given `body` as one of its collision
   geometries. */
  template <typename T>
  static void AddCollisionGeometryToBody(GeometryId id, const Body<T>& body,
                                         MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(body.index() < plant->num_bodies());
    plant->collision_geometries_[body.index()].push_back(id);
  }
};

namespace internal {

/* For the purpose of this test, enable writing FullBodyName to string so that
 failure messages include human readable output (rather than bytestrings). */
std::ostream& operator<<(std::ostream& out, const FullBodyName& name) {
  out << "Model: '" << name.model << "', Body = '" << name.body << "', Geo: '"
      << name.geometry << "', is_unique: " << name.body_name_is_unique
      << ", geometry count: " << name.geometry_count;
  return out;
}

}  // namespace internal

namespace {

/* The fixed number of faces in the test mesh for hydroleastic contact. */
constexpr int kNumFaces = 2;
constexpr int kNumPointPerTri = 3;

/* Creates an arbitrary contact surface representing contact between the
 geometries identified by `id_M` and `id_N`. The surface data (vertex positions,
 pressure field) are largely garbage. The value aren't used in math, only to
 be copied and compared. The underlying mesh's vertex positions can be offset
 by the given `offset` value so that we can distinguish between two "different"
 meshes. */
template <typename T>
ContactSurface<T> MakeContactSurface(GeometryId id_M, GeometryId id_N,
                                     const Vector3<T>& offset) {
  /* Create the surface mesh first. It is simply two triangles (make sure we're
   looping through elements). The position of the vertices is offset by offset.
   The position of the vertices is irrelevant -- the mesh is just a collection
   of doubles that get copied. */
  vector<SurfaceTriangle> faces;
  vector<Vector3<T>> vertices;
  vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5) + offset);
  vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5) + offset);
  vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5) + offset);
  vertices.emplace_back(Vector3<double>(0.5, -0.5, -0.5) + offset);
  faces.emplace_back(0, 1, 2);
  faces.emplace_back(2, 3, 0);
  auto mesh = make_unique<TriangleSurfaceMesh<T>>(move(faces), move(vertices));

  /* Create the "e" field values (i.e., "hydroelastic pressure") - simply
   increasing values at each vertex. */
  vector<T> e_MN(mesh->num_vertices());
  std::iota(e_MN.begin(), e_MN.end(), 1);

  TriangleSurfaceMesh<T>* mesh_pointer = mesh.get();
  EXPECT_EQ(mesh->num_triangles(), kNumFaces);
  return ContactSurface<T>(
      id_M, id_N, move(mesh),
      make_unique<MeshFieldLinear<T, TriangleSurfaceMesh<T>>>(move(e_MN),
                                                              mesh_pointer));
}

/* Makes quadrature data sufficient for the ContactSurface generated by
 MakeContactSurface(). */
template <typename T>
vector<HydroelasticQuadraturePointData<T>> MakeQuadratureData(
    const Vector3<T>& offset) {
  /* We'll pick *three* quadrature points per triangle, to make sure we get
   proper loop iteration. For simplicitly, we'll explicitly define the first
   quadrature point and then express the others in terms of the first. */
  constexpr int kNumPoints = kNumFaces * kNumPointPerTri;
  vector<HydroelasticQuadraturePointData<T>> quadrature_point_data(kNumPoints);

  quadrature_point_data[0].p_WQ = Vector3<double>(1, 3, 5) + offset;
  quadrature_point_data[0].vt_BqAq_W = Vector3<double>(7, 11, 13) + offset;
  quadrature_point_data[0].traction_Aq_W = Vector3<double>(17, 19, 23) + offset;
  quadrature_point_data[0].face_index = 0;
  for (int i = 1; i < kNumPoints; ++i) {
    quadrature_point_data[i].p_WQ = quadrature_point_data[i - 1].p_WQ + offset;
    quadrature_point_data[i].vt_BqAq_W =
        quadrature_point_data[i - 1].vt_BqAq_W + offset;
    quadrature_point_data[i].traction_Aq_W =
        quadrature_point_data[i - 1].traction_Aq_W + offset;
    quadrature_point_data[i].face_index = i / kNumPointPerTri;
  }
  return quadrature_point_data;
}

/* Retrieves a set of fixed geometry ids -- always the same N values for a given
 N. */
template <int N>
const std::array<GeometryId, N>& GetGeometryIds() {
  static bool initialized = false;
  static std::array<GeometryId, N> ids;
  if (!initialized) {
    initialized = true;
    for (int i = 0; i < N; ++i) {
      ids[i] = GeometryId::get_new_id();
    }
  }
  return ids;
}

/* ContactResultsToLcmSystem basically does two things:

  - Upon construction, save a bunch of data about the population of the given
    MultibodyPlant into one or more tables.
  - In calculating its output value (lcm message), combine input ContactResults
    with the stored tables to make the messages.

 The testing exploits this structure directly. To test the constructor, we'll
 use friend access to examine the internal tables directly -- in other words,
 answer the question: given a plant, do we correctly populate the tables?

 Independently, we'll test the output calculation by asking, given a known
 set of tables, and a known contact result, do we get the expected message?

 Finally, we need to test its scalar support: both in constructing a system as
 well as transmogrifying it. This test harness and the supporting functions are
 all designed to support this testing strategy.

 When constructing T-valued ContactResults, we only use constant values. When
 T = symbolic::Expression, we do not have values that are symbolic::Variables.
 As constants, the tests pass as is, with Variables, they would throw (can't
 extract a double value from a Variable without an environment). This throwing
 behavior is not explicitly tested.

 ContactResults contains various vectors of data (e.g., vertex positions,
 quadrature point data, etc.) The test is *assuming* that the order of the
 data in the input is the order of the data on the output. It simplifies the
 test and only fails if the serialization perturbs the order. */
template <typename T>
class ContactResultsToLcmTest : public ::testing::Test {
 protected:
  /* Adds a body with the given `name` to the given `plant` as part of the given
   model instance.

   The constructor of ContactResultsToLcmSystem populates two tables. This
   method aids in building two corresponding tables which contain the expected
   contents. These can be compared directly with the tables in the dut after
   construction.

   The first table (`body_names`) maps body index to a mangled body name.

   The second table (`id_to_body`) maps collision geometry id to a collection of
   related collection of body name data (`ref_name`). The provided FullBodyName,
   `ref_name`, should have the body_name_is_unique and geometry_count fields
   set. This method adds ref_name.geometry_count collision geometries to the
   new body. It inserts one entry into the geometry -> body name map by
   using the `ref_name` and setting the `model` name, `body` name, and
   `geometry` name (using the `namer` function to map id to string).

   @param body_name     The name of the body to add.
   @param model_index   The model instance to which this body will be added.
   @param namer         A functor that turns GeometryId into strings.
   @param plant         The plant to add the body to.
   @param body_names    The ContactResultsToLcmSystem table that maps body
                        index to body name.
   @param id_to_body    The ContactResultsToLcmSystem table that maps
                        GeometryId to FullBodyName.
   @param ref_name      A reference FullBodyName which has already defined
                        .geometry_count and .body_name_is_unique.
   @pre `model_index` is a valid model instance index. */
  void AddBody(const std::string& body_name,
                     ModelInstanceIndex model_index,
                     const function<string(GeometryId)>& namer,
                     MultibodyPlant<T>* plant, vector<string>* body_names,
                     unordered_map<GeometryId, FullBodyName>* id_to_body,
                     FullBodyName ref_name) {
    const auto& body =
        plant->AddRigidBody(body_name, model_index, SpatialInertia<double>());
    /* The expected format based on knowledge of the ContactResultToLcmSystem's
     implementation. */
    body_names->push_back(fmt::format("{}({})", body_name, model_index));

    /* We will *simulate* registering a collision geometry with MbP. Rather than
     instantiating SceneGraph<T> (which we can't even do for symbolic), we'll
     use friend access to shove a GeometryId into MbP's table of known,
     per-body collision geometries. That is sufficient for
     ContactResultsToLcmSystem to add entries to its tables. */
    ref_name.model = plant->GetModelInstanceName(model_index);
    ref_name.body = body.name();
    for (int g = 0; g < ref_name.geometry_count; ++g) {
      const GeometryId g_id = GeometryId::get_new_id();
      MultibodyPlantTester::AddCollisionGeometryToBody(g_id, body, plant);
      ref_name.geometry = namer(g_id);
      id_to_body->insert({g_id, ref_name});
    }
  }

  /* Adds fake point-pair contact results to the given set of contact `results`.
   Also adds names to the given `lcm_system`'s "body names" table so that the
   body indices stored in the point-pair results map to names (if `lcm_system`
   is not nullptr). This doesn't affect the geometry id -> body name map.

   @pre The given `lcm_system` has a *single* entry in its body_names look-up
        table. */
  template <typename U = T>
  static void AddFakePointPairContact(ContactResultsToLcmSystem<U>* lcm_system,
                                      ContactResults<U>* results) {
    if (lcm_system != nullptr) {
      ASSERT_EQ(ContactResultsToLcmTester::get_body_names(lcm_system).size(),
                1);

      ContactResultsToLcmTester::AddToBodyNames(
          lcm_system, {"Body1", "Body2", "Body3", "Unreferenced"});
    }

    /* We don't need to populate the GeometryId->Body map because that is only
     used for hydroelastic contacts. */
    const BodyIndex b0{1};
    const BodyIndex b1{2};
    const BodyIndex b2{3};

    static const never_destroyed<PointPairContactInfo<U>> pair1(
        b0, b1, Vector3<U>{1.1, 2.2, 3.3}, Vector3<U>{4.4, 5.5, 6.6}, 7.7, 8.8,
        PenetrationAsPointPair<U>{
            GeometryId::get_new_id(), GeometryId::get_new_id(),
            Vector3<U>{11.1, 22.2, 33.3}, Vector3<U>{44.4, 55.5, 66.6},
            Vector3<U>{77.7, 88.8, 99.9}, 101.1});
    results->AddContactInfo(pair1.access());
    static const never_destroyed<PointPairContactInfo<U>> pair2(
        b1, b2, Vector3<U>{1.2, 2.3, 3.4}, Vector3<U>{4.5, 5.6, 6.7}, 7.8, 8.9,
        PenetrationAsPointPair<U>{
            GeometryId::get_new_id(), GeometryId::get_new_id(),
            Vector3<U>{11.2, 22.3, 33.4}, Vector3<U>{44.5, 55.6, 66.7},
            Vector3<U>{77.8, 88.9, 99.1}, 101.1});
    results->AddContactInfo(pair2.access());
  }

  /* Adds fake hydro contact results to the given set of contact `results`.
   Also adds entries to the given `lcm_system`'s geometry id -> body name map so
   that the ids stored in the results map to names (if `lcm_system` is not
   nullptr). This doesn't affect the simple body_names vector. */
  template <typename U = T>
  static void AddFakeHydroContact(ContactResultsToLcmSystem<U>* lcm_system,
                                  ContactResults<U>* results) {
    /* Everything here is static because the ContactResults really doesn't own
     any of its data; it keeps pointers to data stored elsewhere. So, we need
     this data to persist, so we keep it function static so it persists through
     the duration of the test. */
    const std::array<GeometryId, 4>& ids = GetGeometryIds<4>();

    if (lcm_system != nullptr) {
      ContactResultsToLcmTester::AddToGeometryBodyMap(
          lcm_system, {{ids[0], {"Model0", "Body0", "Geo0"}},
                       {ids[1], {"Model1", "Body1", "Geo1"}},
                       {ids[2], {"Model2", "Body2", "Geo2"}},
                       {ids[3], {"Model3", "Body3", "Geo3"}},
                       {GeometryId::get_new_id(), {"M", "B", "G"}}});
    }

    /* In creating this fake contact data, what matters *most* is that the body
     indices used map to values in the names we've added above. All other values
     can be meaningless garbage. All that matters is that they are unique values
     such that we can confirm that the right value got written to the right
     field. */
    static const never_destroyed<ContactSurface<U>> surface1(
        MakeContactSurface<U>(ids[0], ids[1], Vector3<U>{1, 2, 3}));
    static const never_destroyed<HydroelasticContactInfo<U>> pair1(
        &surface1.access(),
        SpatialForce<U>(Vector3<U>(1.1, 2.2, 3.3), Vector3<U>(4.4, 5.5, 6.6)),
        MakeQuadratureData<U>(Vector3<U>{1, 2, 3}));
    results->AddContactInfo(&pair1.access());

    static const never_destroyed<ContactSurface<U>> surface2(
        MakeContactSurface<U>(ids[2], ids[3], Vector3<U>{-3, -1, 2}));
    static const never_destroyed<HydroelasticContactInfo<U>> pair2(
        &surface2.access(),
        SpatialForce<U>(Vector3<U>(1.2, 2.3, 3.4), Vector3<U>(4.5, 5.6, 6.7)),
        MakeQuadratureData<U>(Vector3<U>{-3, -1, -2}));
    results->AddContactInfo(&pair2.access());
  }
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd, Expression>;
TYPED_TEST_SUITE(ContactResultsToLcmTest, ScalarTypes);

/* We construct a plant with known contents, instantiate an instance of
 ContactResultsToLcmSystem on it and confirm the internal tables are populated
 as we expect them to be.

 We're exercising the private constructor that takes the geometry-naming
 functor. Accessing that functionality is only *really* available by calling
 ConnectContactResultsToDrakeVisualizer(), but by testing the functor flavor
 of the constructor *here*, we leverage this code for both modes of construction
 and simplify the tests on those functions to simply look for evidence that
 the proper constructor was called (having already shown the constructor to be
 correct). */
TYPED_TEST(ContactResultsToLcmTest, Constructor) {
  using T = TypeParam;

  for (bool use_custom_names : {true, false}) {
    /* These mirror the internal data tables in ContactResultsToLcmSystem. We'll
     populate them as we populate MBP and confirm that the resulting tables
     match the tables we build by hand. */
    unordered_map<GeometryId, FullBodyName> expected_geo_body_map;
    /* The world body will always be the first listed. */
    vector<string> expected_body_names{{"WorldBody(0)"}};

    auto namer = [use_custom_names](GeometryId id) {
      if (use_custom_names) {
        /* Create an arbitrary name unique to this test. */
        return fmt::format("CustomTestId({})", id);
      }
      /* Reproduce the expected default name for ContactResultsToLcmSystem. */
      return fmt::format("Id({})", id);
    };

    MultibodyPlant<T> plant(0.0);
    /* Bodies 1 and 2 go to the same model instance. Body 3 goes to its own.
      Body 1 gets two geometries (to test the geometry_count field). Bodies 1
      and 3 have the same name (to test body_name_is_unique_field).  */
    const ModelInstanceIndex model12 = plant.AddModelInstance("JustForBody12");

    FullBodyName ref_name;

    ref_name.body_name_is_unique = false;
    ref_name.geometry_count = 2;
    this->AddBody("dupe_name", model12, namer, &plant, &expected_body_names,
                  &expected_geo_body_map, ref_name);

    ref_name.body_name_is_unique = true;
    ref_name.geometry_count = 1;
    this->AddBody("body2", model12, namer, &plant, &expected_body_names,
                  &expected_geo_body_map, ref_name);

    const ModelInstanceIndex model3 = plant.AddModelInstance("JustForBody3");
    ref_name.body_name_is_unique = false;
    this->AddBody("dupe_name", model3, namer, &plant, &expected_body_names,
                  &expected_geo_body_map, ref_name);

    plant.Finalize();

    SCOPED_TRACE(use_custom_names ? "Using custom names"
                                  : "Using default names");
    /* Construction should populate tables about bodies. */
    unique_ptr<ContactResultsToLcmSystem<T>> lcm{};
    if (use_custom_names) {
      lcm = ContactResultsToLcmTester::Make(plant, namer);
    } else {
      /* Rather than simply passing in `nullptr`, we want to test the default-
       value spelling of the constructor. */
      lcm = ContactResultsToLcmTester::Make(plant);
    }

    /* Examine the constructed tables. */
    const auto& body_names =
        ContactResultsToLcmTester::get_body_names(lcm.get());
    const auto& id_to_body_map =
        ContactResultsToLcmTester::get_geometry_id_to_body_map(lcm.get());
    EXPECT_EQ(body_names, expected_body_names);
    EXPECT_EQ(id_to_body_map, expected_geo_body_map);

    /* We'll further confirm that the system has its default name and ports
     available. */
    EXPECT_EQ(lcm->get_name(), "ContactResultsToLcmSystem");
    EXPECT_NO_THROW(lcm->get_contact_result_input_port());
    EXPECT_NO_THROW(lcm->get_lcm_message_output_port());
  }
}

/* Confirms that empty ContactResults produces an empty message. */
TYPED_TEST(ContactResultsToLcmTest, EmptyContactResults) {
  using T = TypeParam;

  /* We're not going to populate the plant, because we're going to set the
   internal tables by hand and fix the input for the context. */
  MultibodyPlant<T> plant(0.0);
  plant.Finalize();
  ContactResultsToLcmSystem<T> lcm(plant);
  const unique_ptr<Context<T>> context = lcm.AllocateContext();

  const double kTime = 1.5;
  context->SetTime(kTime);
  lcm.get_contact_result_input_port().FixValue(context.get(),
                                               ContactResults<T>{});

  /* We'll do this twice, once with *empty* internal tables, and once with
   tables with values. We want to make sure that empty contact, even with
   populated tables is not a problem. */
  auto confirm_empty = [&lcm, &context = *context, kTime]() {
    const auto& message =
        lcm.get_lcm_message_output_port()
            .template Eval<lcmt_contact_results_for_viz>(context);

    // Message time is an integer number of microseconds.
    const double kTimeMicroSec = static_cast<int64_t>(kTime * 1e6);
    EXPECT_EQ(message.timestamp, kTimeMicroSec);
    EXPECT_EQ(message.num_point_pair_contacts, 0);
    EXPECT_EQ(message.point_pair_contact_info.size(), 0);
    EXPECT_EQ(message.num_hydroelastic_contacts, 0);
    EXPECT_EQ(message.hydroelastic_contacts.size(), 0);
  };

  {
    SCOPED_TRACE("With empty tables");
    confirm_empty();
  }

  /* Add some content to the tables and try again. The actual content is
   arbitrary garbage. */
  {
    ContactResultsToLcmTester::AddToBodyNames(
        &lcm, {"A body", "Another body", "and more"});
    ContactResultsToLcmTester::AddToGeometryBodyMap(
        &lcm, {{GeometryId::get_new_id(), {"A model", "A body", "A geometry"}},
               {GeometryId::get_new_id(), {"names", "don't", "matter"}}});
    SCOPED_TRACE("With populated tables");
    confirm_empty();
  }
}

/* Tests the case where ContactResults contains *only* point-pair data. This
 test bears primary responsibility to make sure that point pair data is
 serialized correctly.

 Translation of point pair contact results to lcm message is straightforward.
 There are *three* things that this system does in the process:

   - Extracts double values from T-Valued quantities.
   - Looks up body names based on *body indices*.
   - Writes various Vector3 quantities (forces, positions, etc. -- all Vector3-
     valued data, but with different semantics). We'll want to make sure that
     each Vector3-valued quantity gets mapped to the right part of the message,
     because type checking isn't going to do it for us.

 As such, we don't need much for this test. We'll do two point-pair contacts
 with unique values to confirm successful iteration and copying. */
TYPED_TEST(ContactResultsToLcmTest, PointPairContactOnly) {
  using T = TypeParam;

  /* We're not going to populate the plant, because we're going to set the
   internal tables by hand and fix the input for the context. */
  MultibodyPlant<T> plant(0.0);
  plant.Finalize();
  ContactResultsToLcmSystem<T> lcm(plant);
  const unique_ptr<Context<T>> context = lcm.AllocateContext();

  /* The time *also* gets set inside the per-point-pair messages. We want to
   detect that. */
  const double kTime = 1.5;
  context->SetTime(kTime);
  ContactResults<T> contacts;
  this->AddFakePointPairContact(&lcm, &contacts);
  lcm.get_contact_result_input_port().FixValue(context.get(), contacts);

  const auto& message =
      lcm.get_lcm_message_output_port()
          .template Eval<lcmt_contact_results_for_viz>(*context);

  EXPECT_EQ(message.num_point_pair_contacts, 2);
  EXPECT_EQ(message.point_pair_contact_info.size(), 2);
  EXPECT_EQ(message.num_hydroelastic_contacts, 0);
  EXPECT_EQ(message.hydroelastic_contacts.size(), 0);

  /* Test the message for a point pair contact against the input point pair
   data. */
  const auto& body_names = ContactResultsToLcmTester::get_body_names(&lcm);
  auto confirm_message = [kTime, &body_names](
                             const lcmt_contact_results_for_viz& message_in,
                             const ContactResults<T>& contacts_in, int i) {
    // Message time is an integer number of microseconds.
    const double kTimeMicroSec = static_cast<int64_t>(kTime * 1e6);
    const auto& pair_message = message_in.point_pair_contact_info[i];
    const auto& pair_data = contacts_in.point_pair_contact_info(i);

    EXPECT_EQ(pair_message.timestamp, kTimeMicroSec);
    EXPECT_EQ(pair_message.body1_name, body_names[pair_data.bodyA_index()]);
    EXPECT_EQ(pair_message.body2_name, body_names[pair_data.bodyB_index()]);
    // clang-format off
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.contact_point),
        ExtractDoubleOrThrow(pair_data.contact_point())));
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.contact_force),
        ExtractDoubleOrThrow(pair_data.contact_force())));
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.normal),
        ExtractDoubleOrThrow(pair_data.point_pair().nhat_BA_W)));
    // clang-format on
    /* None of the rest of the pair data makes it to the message. */
  };

  for (int c = 0; c < 2; ++c) {
    SCOPED_TRACE(fmt::format("Penetration point pair {} out of 2", (c + 1)));
    confirm_message(message, contacts, c);
  }
}

/* Tests the case where ContactResults contains *only* hydroleastic data. This
 test bears primary responsibility to make sure that hydroleastic data is
 serialized correctly.

 Translation of hydroleastic contact results to lcm message is straightforward.
 There are *three* things that this system does in the process:

   - Extracts double values from T-Valued quantities.
   - Looks up body names based on geometry ids.
   - Writes various Vector3 quantities (forces, positions, etc. -- all Vector3-
     valued data, but with different semantics). We'll want to make sure that
     each Vector3-valued quantity gets mapped to the right part of the message,
     because type checking isn't going to do it for us.

 As such, we don't need much for this test. We'll do two hydroelastic contacts
 with unique values to confirm successful iteration and copying. */
TYPED_TEST(ContactResultsToLcmTest, HydroContactOnly) {
  using T = TypeParam;

  /* We're not going to populate the plant, because we're going to set the
   internal tables by hand and fix the input for the context. */
  MultibodyPlant<T> plant(0.0);
  plant.Finalize();
  ContactResultsToLcmSystem<T> lcm(plant);
  const unique_ptr<Context<T>> context = lcm.AllocateContext();

  /* Hydroelastic contact doesn't store time; so we'll leave it alone. */
  ContactResults<T> contacts;
  this->AddFakeHydroContact(&lcm, &contacts);
  lcm.get_contact_result_input_port().FixValue(context.get(), contacts);

  const auto& message =
      lcm.get_lcm_message_output_port()
          .template Eval<lcmt_contact_results_for_viz>(*context);

  EXPECT_EQ(message.num_point_pair_contacts, 0);
  EXPECT_EQ(message.point_pair_contact_info.size(), 0);
  EXPECT_EQ(message.num_hydroelastic_contacts, 2);
  EXPECT_EQ(message.hydroelastic_contacts.size(), 2);

  /* Test the message for hydro contact against the input hydro data. */
  const auto& geo_to_body_map =
      ContactResultsToLcmTester::get_geometry_id_to_body_map(&lcm);
  auto confirm_message = [&geo_to_body_map](
                             const lcmt_contact_results_for_viz& message_in,
                             const ContactResults<T>& contacts_in, int i) {
    const auto& pair_message = message_in.hydroelastic_contacts[i];
    const auto& pair_data = contacts_in.hydroelastic_contact_info(i);
    const auto& surface = pair_data.contact_surface();
    const auto& mesh = surface.tri_mesh_W();
    const auto& field = surface.tri_e_MN();

    const auto& name1 = geo_to_body_map.at(surface.id_M());
    EXPECT_EQ(pair_message.body1_name, name1.body);
    EXPECT_EQ(pair_message.model1_name, name1.model);
    EXPECT_EQ(pair_message.geometry1_name, name1.geometry);
    EXPECT_EQ(pair_message.body1_unique, name1.body_name_is_unique);
    EXPECT_EQ(pair_message.collision_count1, name1.geometry_count);

    const auto& name2 = geo_to_body_map.at(surface.id_N());
    EXPECT_EQ(pair_message.body2_name, name2.body);
    EXPECT_EQ(pair_message.model2_name, name2.model);
    EXPECT_EQ(pair_message.geometry2_name, name2.geometry);
    EXPECT_EQ(pair_message.body2_unique, name2.body_name_is_unique);
    EXPECT_EQ(pair_message.collision_count2, name2.geometry_count);

    /* Mesh aggregate results: centroid, force, moment. */
    // clang-format off
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.centroid_W),
        ExtractDoubleOrThrow(mesh.centroid())));
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.force_C_W),
        ExtractDoubleOrThrow(pair_data.F_Ac_W().translational())));
    EXPECT_TRUE(CompareMatrices(
        Vector3<double>(pair_message.moment_C_W),
        ExtractDoubleOrThrow(pair_data.F_Ac_W().rotational())));
    // clang-format on

    /* Compare meshes and pressure fields. */

    /* Confirm vertices (count and values) and per-vertex pressure values. */
    EXPECT_EQ(static_cast<int>(pair_message.pressure.size()),
              mesh.num_vertices());
    EXPECT_EQ(pair_message.num_vertices, mesh.num_vertices());
    EXPECT_EQ(static_cast<int>(pair_message.p_WV.size()), mesh.num_vertices());
    for (int v = 0; v < mesh.num_vertices(); ++v) {
      const auto& point_WV = pair_message.p_WV[v];
      const Vector3<double> p_WV_message(point_WV.x, point_WV.y, point_WV.z);
      EXPECT_TRUE(CompareMatrices(p_WV_message, mesh.vertex(v)));
      EXPECT_EQ(pair_message.pressure[v], field.EvaluateAtVertex(v));
    }

    /* Confirm faces. Each triangle produces a sequence that looks like:
    // [3, i0, i1, i2] in the face data. Confirm size and contents. */
    ASSERT_EQ(pair_message.poly_data_int_count, mesh.num_triangles() * 4);
    ASSERT_EQ(pair_message.poly_data.size(), mesh.num_triangles() * 4);

    int index = -1;
    for (int f = 0; f < mesh.num_triangles(); ++f) {
      const auto& tri = mesh.element(f);
      ASSERT_EQ(pair_message.poly_data[++index], 3);
      ASSERT_EQ(pair_message.poly_data[++index], tri.vertex(0));
      ASSERT_EQ(pair_message.poly_data[++index], tri.vertex(1));
      ASSERT_EQ(pair_message.poly_data[++index], tri.vertex(2));
    }

    /* Compare quadrature data. */
    const auto& data_quads = pair_data.quadrature_point_data();
    const auto& message_quads = pair_message.quadrature_point_data;
    EXPECT_EQ(pair_message.num_quadrature_points, data_quads.size());
    EXPECT_EQ(message_quads.size(), data_quads.size());
    EXPECT_EQ(pair_message.num_quadrature_points,
              mesh.num_triangles() * kNumPointPerTri);
    for (int q = 0; q < static_cast<int>(data_quads.size()); ++q) {
      // clang-format off
      EXPECT_TRUE(CompareMatrices(
        Vector3<double>(message_quads[q].p_WQ),
        ExtractDoubleOrThrow(data_quads[q].p_WQ)));
      EXPECT_TRUE(CompareMatrices(
        Vector3<double>(message_quads[q].vt_BqAq_W),
        ExtractDoubleOrThrow(data_quads[q].vt_BqAq_W)));
      EXPECT_TRUE(CompareMatrices(
        Vector3<double>(message_quads[q].traction_Aq_W),
        ExtractDoubleOrThrow(data_quads[q].traction_Aq_W)));
      // clang-format on
    }
  };

  for (int c = 0; c < 2; ++c) {
    SCOPED_TRACE(fmt::format("Hydro contact {} out of 2", (c + 1)));
    confirm_message(message, contacts, c);
  }
}

/* Tests the case where ContactResults contains mixed data. This test assumes
 that point and hydro data, if represented *at all*, are represented correctly.
 It just attempts to confirm that the two result types don't interfere.
 Correctness of the serialization of each type relies on previous tests. */
TYPED_TEST(ContactResultsToLcmTest, MixedContactData) {
  using T = TypeParam;

  /* We're not going to populate the plant, because we're going to set the
   internal tables by hand and fix the input for the context. */
  MultibodyPlant<T> plant(0.0);
  plant.Finalize();
  ContactResultsToLcmSystem<T> lcm(plant);
  const unique_ptr<Context<T>> context = lcm.AllocateContext();

  ContactResults<T> contacts;
  this->AddFakePointPairContact(&lcm, &contacts);
  this->AddFakeHydroContact(&lcm, &contacts);
  lcm.get_contact_result_input_port().FixValue(context.get(), contacts);

  const auto& message =
      lcm.get_lcm_message_output_port()
          .template Eval<lcmt_contact_results_for_viz>(*context);

  EXPECT_EQ(message.num_point_pair_contacts, 2);
  EXPECT_EQ(message.point_pair_contact_info.size(), 2);
  EXPECT_EQ(message.num_hydroelastic_contacts, 2);
  EXPECT_EQ(message.hydroelastic_contacts.size(), 2);
  /* Knowing that we have the right *numbers* of contact results suffices. We
   assume if the serialization got that far, it did the right thing at the
   detail level based on previous tests. */
}

/* Previous tests confirmed that construction ContactResultsToLcmSystem for
 various T works. This confirms that transmogrifying from double likewise
 works.

 ContactResultsToLcmSystem provides a private Equal() method to compare systems.
 This is a good minimum condition, but we also use a smoke test that things are
 correct -- we want to make sure the transmogrified result is *functional*.

 Starting with a double-valued system, we'll transmogrify it, allocate a
 context, fix a non-empty contact result to the input port, evaluate the output
 port, and make sure things got serialized. */
TYPED_TEST(ContactResultsToLcmTest, Transmogrifcation) {
  using T = TypeParam;

  MultibodyPlant<double> plant(0.0);
  plant.Finalize();
  auto custom_names = [](GeometryId id) {
    return fmt::format("String that must be copied to match {}", id);
  };
  auto lcm_double = ContactResultsToLcmTester::Make(plant, custom_names);
  lcm_double->set_name("Ad hoc name");

  /* We don't care about double-valued results, we're just using it to
   populate the tables in lcm_double. These will get copied over. */
  ContactResults<double> contacts_double;
  this->AddFakePointPairContact(lcm_double.get(), &contacts_double);
  this->AddFakeHydroContact(lcm_double.get(), &contacts_double);

  auto system_T = [&double_source = *lcm_double]() {
    if constexpr (std::is_same_v<T, double>) {
      /* We support AutoDiffXd -> double. So, when T is double, we'll convert to
       AutoDiffXd and *back*. Obviously if double -> AutoDiffXd is broken both
       that specific test and this test will fail. */
      return double_source.ToAutoDiffXd()->template ToScalarType<double>();
    } else {
      return double_source.template ToScalarType<T>();
    }
  }();
  ContactResultsToLcmSystem<T>* lcm_T =
      dynamic_cast<ContactResultsToLcmSystem<T>*>(system_T.get());
  ASSERT_NE(lcm_T, nullptr);

  EXPECT_TRUE(ContactResultsToLcmTester::Equals(*lcm_double, *lcm_T));

  ContactResults<T> contacts;
  this->template AddFakePointPairContact<T>(nullptr, &contacts);
  this->template AddFakeHydroContact<T>(nullptr, &contacts);
  const unique_ptr<Context<T>> context = lcm_T->AllocateContext();
  lcm_T->get_contact_result_input_port().FixValue(context.get(), contacts);

  const auto& message =
      lcm_T->get_lcm_message_output_port()
          .template Eval<lcmt_contact_results_for_viz>(*context);

  EXPECT_EQ(message.num_point_pair_contacts, 2);
  EXPECT_EQ(message.point_pair_contact_info.size(), 2);
  EXPECT_EQ(message.num_hydroelastic_contacts, 2);
  EXPECT_EQ(message.hydroelastic_contacts.size(), 2);
}

/* There are four overloads of ConnectContactResultsToDrakeVisualizer(). They
 differ along two axes:

   - Does the ContactResultsToLcmSystem instance connect directly to the plant
     or to some arbitrary passed OutputPort?
   - Does ContactResultsToLcmSystem use default geometry names or does it get
     geometry names from a given SceneGraph instance.

 The test fixture is built to facilitate tests structured along those axes. Each
 test comprises four parts:

   1. Construct a diagram (see ConfigureDiagram()).
   2. Invoke a particular overload (found in each TEST_F).
   3. Confirm the returned value is of expected type with documented properties
      (see ExpectValidPublisher()).
   4. Confirm geometry names are as expected (see ExpectGeometryNameSemantics().
 */
class ConnectVisualizerTest : public ::testing::Test {
 protected:
  void AddPlantAndSceneGraphAndOneLink(DiagramBuilder<double>* builder) {
    auto system_pair = AddMultibodyPlantSceneGraph(builder, 0.0);
    plant_ = &system_pair.plant;
    scene_graph_ = &system_pair.scene_graph;

    const auto& body = plant_->AddRigidBody("link", SpatialInertia<double>());
    plant_->RegisterCollisionGeometry(body, {}, Sphere(1.0), kGeoName,
                                      CoulombFriction<double>{});
    plant_->Finalize();
  }

  void ConfigureDiagram(bool is_nested) {
    if (is_nested) {
      /* We'll treat the MBP-SG pair as a nested diagram so we can test all
       overloads of the DUT. This means, exporting the contact port out of the
       diagram. */
      DiagramBuilder<double> inner_builder;
      AddPlantAndSceneGraphAndOneLink(&inner_builder);
      inner_builder.ExportOutput(plant_->get_contact_results_output_port(),
                                 "contact_results");
      auto diagram = builder_.AddSystem(inner_builder.Build());
      contact_results_port_ = &diagram->GetOutputPort("contact_results");
    } else {
      AddPlantAndSceneGraphAndOneLink(&builder_);
    }
  }

  /* Confirms that the publisher pointer is non-null and has been configured
   with the given publication period.

   When no publication period is given, we check that the default value from the
   cc file made it through.  We don't specifically care that this is 64 Hz, just
   that it's some sensible default.  If the cc file changes, we should update
   the value here as well.
  */
  void ExpectValidPublisher(systems::lcm::LcmPublisherSystem* publisher,
                            double expected_publish_period = 1.0 / 64) {
    /* Confirm that we get a non-null result. */
    ASSERT_NE(publisher, nullptr);

    /* Check that the publishing event was set as documented. */
    EXPECT_EQ(publisher->get_publish_period(), expected_publish_period);
  }

  /* Confirms that the names for geometries stored in the
   ContactResultsToLcmSystem are as expepcted (default or scene_graph as
   indicated). */
  void ExpectGeometryNameSemantics(bool expect_default_names) {
    /* Grab the contact results to lcm system and confirm the namer has
     correctly handled. The name should either be the tester constant (kGeoName)
     or Id(\d+). */
    for (auto* system : builder_.GetMutableSystems()) {
      auto* lcm = dynamic_cast<ContactResultsToLcmSystem<double>*>(system);
      if (lcm != nullptr) {
        const auto& id_to_body_map =
            ContactResultsToLcmTester::get_geometry_id_to_body_map(lcm);
        ASSERT_EQ(id_to_body_map.size(), 1);
        const auto& [id, name] = *id_to_body_map.begin();
        if (expect_default_names) {
          EXPECT_EQ(name.geometry, fmt::format("Id({})", id));
        } else {
          EXPECT_EQ(name.geometry, kGeoName);
        }
        return;
      }
    }
    GTEST_FAIL() << "The diagram builder did not have an instance of "
                    "ContactResultsToLcmSystem.";
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  const systems::OutputPort<double>* contact_results_port_{};
  static constexpr char kGeoName[] = "test_sphere";
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_F(ConnectVisualizerTest, DeprecatedConnectToPlantDefaultNames) {
  ConfigureDiagram(false /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(&builder_, *plant_);
  ExpectValidPublisher(publisher);
  ExpectGeometryNameSemantics(true /* expect_default_names */);
}
#pragma GCC diagnostic push

TEST_F(ConnectVisualizerTest, ConnectToPlantSceneGraphNames) {
  ConfigureDiagram(false /* is_nested */);
  auto* publisher =
      ConnectContactResultsToDrakeVisualizer(&builder_, *plant_, *scene_graph_);
  ExpectValidPublisher(publisher);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_F(ConnectVisualizerTest, DeprecatedConnectToPortDefaultNames) {
  ConfigureDiagram(true /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *plant_, *contact_results_port_);
  ExpectValidPublisher(publisher);
  ExpectGeometryNameSemantics(true /* expect_default_names */);
}
#pragma GCC diagnostic pop

TEST_F(ConnectVisualizerTest, ConnectToPortSceneGraphNames) {
  ConfigureDiagram(true /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *plant_, *scene_graph_, *contact_results_port_);
  ExpectValidPublisher(publisher);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_F(ConnectVisualizerTest, DeprecatedConnectToPlantDefaultNamesWithPeriod) {
  ConfigureDiagram(false /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *plant_, nullptr, 0.5);
  ExpectValidPublisher(publisher, 0.5);
  ExpectGeometryNameSemantics(true /* expect_default_names */);
}
#pragma GCC diagnostic pop

TEST_F(ConnectVisualizerTest, ConnectToPlantSceneGraphNamesWithPeriod) {
  ConfigureDiagram(false /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *plant_, *scene_graph_, nullptr, 0.5);
  ExpectValidPublisher(publisher, 0.5);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_F(ConnectVisualizerTest, DeprecatedConnectToPortDefaultNamesWithPeriod) {
  ConfigureDiagram(true /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *plant_, *contact_results_port_, nullptr, 0.5);
  ExpectValidPublisher(publisher, 0.5);
  ExpectGeometryNameSemantics(true /* expect_default_names */);
}
#pragma GCC diagnostic pop

TEST_F(ConnectVisualizerTest, ConnectToPortSceneGraphNamesWithPeriod) {
  ConfigureDiagram(true /* is_nested */);
  auto* publisher = ConnectContactResultsToDrakeVisualizer(
      &builder_, *plant_, *scene_graph_, *contact_results_port_, nullptr, 0.5);
  ExpectValidPublisher(publisher, 0.5);
  ExpectGeometryNameSemantics(false /* expect_default_names */);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
