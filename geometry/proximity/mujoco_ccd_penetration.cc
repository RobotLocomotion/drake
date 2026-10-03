#include "drake/geometry/proximity/mujoco_ccd_penetration.h"

#include <algorithm>
#include <cfloat>
#include <limits>
#include <utility>

// MuJoCo's native convex collision detection (from @mujoco_internal). The
// header declares mjc_ccd()/mjc_ccdSize() and the mjCCDObj/mjCCDConfig/
// mjCCDStatus structs, with C linkage.
#include <engine/engine_collision_gjk.h>
#include <engine/engine_util_errmem.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

// mjc_ccd() references these two support functions from MuJoCo's
// engine_collision_convex.c (its sphere/capsule shrink-and-inflate path),
// which Drake does not compile. Drake only ever passes mjGEOM_MESH objects,
// so the referencing branch is unreachable at runtime; these exact ports of
// the MuJoCo definitions satisfy the linker (and behave correctly should
// they ever be reached). They are hidden so libdrake.so does not export the
// (non-namespaced) names.
extern "C" __attribute__((visibility("hidden"))) void mjc_pointSupport(
    mjtNum res[3], mjCCDObj* obj, const mjtNum[3]) {
  res[0] = obj->pos[0];
  res[1] = obj->pos[1];
  res[2] = obj->pos[2];
}

extern "C" __attribute__((visibility("hidden"))) void mjc_lineSupport(
    mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjtNum* mat = obj->mat;
  const mjtNum* pos = obj->pos;
  const mjtNum length = obj->size[1];
  const mjtNum dot = mat[2] * dir[0] + mat[5] * dir[1] + mat[8] * dir[2];
  const mjtNum scl = dot >= 0 ? length : -length;
  res[0] = mat[2] * scl + pos[0];
  res[1] = mat[5] * scl + pos[1];
  res[2] = mat[8] * scl + pos[2];
}

namespace drake {
namespace geometry {
namespace internal {
namespace {

/* MuJoCo's model-compiler defaults for the CCD pipeline (mjOption's
 ccd_tolerance and ccd_iterations in engine_init.c). */
constexpr double kCcdTolerance = 1e-6;
constexpr int kCcdIterations = 35;

/* MuJoCo produces at most 4 contacts for a mesh-mesh pair on the native
 (zero-margin) path: larger clipped contact polygons are pruned to the
 maximum-area quadrilateral (see maxContacts() in MuJoCo's
 engine_collision_convex.c). */
constexpr int kMaxContacts = 4;

/* MuJoCo's EPA reports a degenerate polytope through mju_warning(). MuJoCo's
 default handler prints to stderr and appends to a file named MUJOCO_LOG.TXT
 in the process's working directory, which a Drake simulation must not do, so
 the handler pointer is redirected here on first use. The pointer is a global
 in Drake's private copy of MuJoCo (compiled with hidden visibility), so
 redirecting it does not affect any other MuJoCo library in the process. The
 warned-about cases are ones this file already tolerates (the affected points
 come back non-finite and are dropped), so they are logged at debug level. */
void LogMujocoWarning(const char* message) {
  drake::log()->debug("MuJoCo convex collision detection: {}", message);
}

void InstallMujocoWarningHandler() {
  static const bool installed = []() {
    mju_user_warning = &LogMujocoWarning;
    return true;
  }();
  unused(installed);
}

/* Support function over the precomputed float vertices; a faithful port of
 MuJoCo's mjc_meshSupport() (linear scan; the hill-climbing variant needs
 mjModel graph data that we don't build). Records the winning vertex index in
 obj->vertindex — EPA's duplicate-support detection and the multi-point
 feature identification both depend on it. */
void DrakeMeshSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjtNum* mat = obj->mat;
  const mjtNum* pos = obj->pos;
  const float* verts = obj->data.mesh.vert;
  const int nverts = obj->data.mesh.nvert;

  // local_dir = matᵀ * dir (mat is row-major world-from-mesh rotation).
  const mjtNum local_dir[3] = {
      mat[0] * dir[0] + mat[3] * dir[1] + mat[6] * dir[2],
      mat[1] * dir[0] + mat[4] * dir[1] + mat[7] * dir[2],
      mat[2] * dir[0] + mat[5] * dir[1] + mat[8] * dir[2]};

  mjtNum max = -FLT_MAX;
  int imax = 0;

  // Seed with the previous winner (only affects tie-breaking; MuJoCo does the
  // same in mjc_meshSupport).
  if (obj->vertindex >= 0) {
    imax = obj->vertindex;
    const float* v = verts + 3 * imax;
    max = local_dir[0] * v[0] + local_dir[1] * v[1] + local_dir[2] * v[2];
  }

  for (int i = 0; i < nverts; ++i) {
    const float* v = verts + 3 * i;
    const mjtNum vdot =
        local_dir[0] * v[0] + local_dir[1] * v[1] + local_dir[2] * v[2];
    if (vdot > max) {
      max = vdot;
      imax = i;
    }
  }
  obj->vertindex = imax;

  const float* v = verts + 3 * imax;
  res[0] = mat[0] * v[0] + mat[1] * v[1] + mat[2] * v[2] + pos[0];
  res[1] = mat[3] * v[0] + mat[4] * v[1] + mat[5] * v[2] + pos[1];
  res[2] = mat[6] * v[0] + mat[7] * v[1] + mat[8] * v[2] + pos[2];
}

/* Center function used to seed the query. MuJoCo returns the geom position
 (its mesh geoms are re-centered at compile time so the position is interior);
 our mesh frames are arbitrary, so we use the hull's vertex centroid, stashed
 in obj->size (unused for mjGEOM_MESH). */
void DrakeMeshCenter(mjtNum res[3], const mjCCDObj* obj) {
  const mjtNum* mat = obj->mat;
  const mjtNum* pos = obj->pos;
  const mjtNum* c = obj->size;
  res[0] = mat[0] * c[0] + mat[1] * c[1] + mat[2] * c[2] + pos[0];
  res[1] = mat[3] * c[0] + mat[4] * c[1] + mat[5] * c[2] + pos[1];
  res[2] = mat[6] * c[0] + mat[7] * c[1] + mat[8] * c[2] + pos[2];
}

void InitCcdObj(const MujocoCcdMeshData& mesh,
                const math::RigidTransformd& X_WG, mjCCDObj* obj) {
  *obj = mjCCDObj{};
  obj->geom = 0;
  obj->geom_type = mjGEOM_MESH;
  obj->vertindex = -1;
  obj->meshindex = -1;
  obj->flex = -1;
  obj->elem = -1;
  obj->vert = -1;
  obj->margin = 0;
  obj->rotate[0] = 1;  // Identity quaternion (unused for geoms).

  const Eigen::Matrix3d R = X_WG.rotation().matrix();
  for (int r = 0; r < 3; ++r) {
    obj->pos[r] = X_WG.translation()[r];
    for (int c = 0; c < 3; ++c) {
      obj->mat[3 * r + c] = R(r, c);
    }
  }
  // The hull centroid rides in size[] for DrakeMeshCenter().
  for (int r = 0; r < 3; ++r) obj->size[r] = mesh.centroid[r];

  obj->data.mesh.nvert = mesh.nvert;
  obj->data.mesh.mesh_polynum = mesh.polynum;
  obj->data.mesh.vert = mesh.vert.data();
  obj->data.mesh.mpolymapadr = mesh.polymapadr.data();
  obj->data.mesh.mpolymapnum = mesh.polymapnum.data();
  obj->data.mesh.polymap = mesh.polymap.data();
  obj->data.mesh.polyvertadr = mesh.polyvertadr.data();
  obj->data.mesh.polyvertnum = mesh.polyvertnum.data();
  obj->data.mesh.polyvert = mesh.polyvert.data();
  obj->data.mesh.polynormal = mesh.polynormal.data();
  obj->data.mesh.graph = nullptr;  // Linear-scan support; no hill climbing.
  obj->data.mesh.extrema = nullptr;
  obj->center = &DrakeMeshCenter;
  obj->support = &DrakeMeshSupport;
}

}  // namespace

bool ComputeMujocoMultipointPenetration(
    const MujocoCcdMeshData& mesh_A, const math::RigidTransformd& X_WA,
    const MujocoCcdMeshData& mesh_B, const math::RigidTransformd& X_WB,
    GeometryId id_A, GeometryId id_B, std::vector<double>* scratch,
    std::vector<PenetrationAsPointPair<double>>* point_pairs) {
  DRAKE_DEMAND(scratch != nullptr);
  DRAKE_DEMAND(point_pairs != nullptr);
  static_assert(sizeof(mjtNum) == sizeof(double),
                "Drake requires MuJoCo compiled with double precision.");
  InstallMujocoWarningHandler();

  mjCCDObj obj_A, obj_B;
  InitCcdObj(mesh_A, X_WA, &obj_A);
  InitCcdObj(mesh_B, X_WB, &obj_B);

  mjCCDConfig config;
  config.max_iterations = kCcdIterations;
  config.tolerance = kCcdTolerance;
  config.max_contacts = kMaxContacts;
  config.dist_cutoff = 0;  // Penetration only; no distance recovery.
  config.npolygonmax = std::max(mesh_A.npolygonmax, mesh_B.npolygonmax);
  config.nmeshdegmax = std::max(mesh_A.nmeshdegmax, mesh_B.nmeshdegmax);
  const size_t bytes =
      mjc_ccdSize(config.npolygonmax, config.nmeshdegmax, kCcdIterations);
  // The scratch vector's element type is double so that the buffer satisfies
  // MuJoCo's 8-byte alignment requirement.
  scratch->resize((bytes + sizeof(double) - 1) / sizeof(double));
  config.buffer = scratch->data();

  mjCCDStatus status{};
  const mjtNum dist = mjc_ccd(&config, &status, &obj_A, &obj_B);
  // A positive distance is a definitive "separated". A distance of exactly
  // zero is not: GJK also reports zero when it stalls at its starting point
  // because the two vertex centroids coincide (EPA is then skipped), so that
  // case falls through to the loop below, appends nothing, and is reported
  // as unresolved.
  if (dist > 0) return true;

  constexpr double kEps = std::numeric_limits<double>::epsilon();
  const int first_new = static_cast<int>(point_pairs->size());
  for (int i = 0; i < status.nx; ++i) {
    // MuJoCo reports a signed distance per witness pair; negative means
    // penetration. Guard against osculation just like Drake's existing
    // point-pair fallback does. The comparison is written so that a NaN
    // distance (EPA can degenerate on very deep penetrations) is also
    // dropped rather than poisoning downstream solvers, and the witness
    // points get the same defense.
    const double depth = -status.dist[i];
    if (!(depth > kEps)) continue;
    PenetrationAsPointPair<double> pair;
    pair.id_A = id_A;
    pair.id_B = id_B;
    pair.p_WCa = Eigen::Vector3d(status.x1[3 * i + 0], status.x1[3 * i + 1],
                                 status.x1[3 * i + 2]);
    pair.p_WCb = Eigen::Vector3d(status.x2[3 * i + 0], status.x2[3 * i + 1],
                                 status.x2[3 * i + 2]);
    if (!pair.p_WCa.allFinite() || !pair.p_WCb.allFinite()) continue;
    // In penetration the witness on A lies inside B and vice versa, so the
    // witness separation p_WCb - p_WCa has length `depth` and points out of
    // B into A; normalizing it yields nhat_BA_W and satisfies Drake's
    // invariant depth = (p_WCb - p_WCa) ⋅ nhat_BA_W.
    pair.nhat_BA_W = (pair.p_WCb - pair.p_WCa).normalized();
    // Degenerate manifold recovery can emit coincident witness points with a
    // finite plane distance, which normalizes to NaN; drop those too.
    if (!pair.nhat_BA_W.allFinite()) continue;
    pair.depth = depth;
    // Degenerate clipping can also emit (near-)coincident contact points.
    // MuJoCo's regularized solver tolerates the duplicated constraint;
    // Drake's contact solvers should not be handed a singular manifold, so
    // keep only the first of any coincident points.
    bool duplicate = false;
    for (int j = first_new; j < static_cast<int>(point_pairs->size()); ++j) {
      if (((*point_pairs)[j].p_WCa - pair.p_WCa).squaredNorm() < 1e-16) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    point_pairs->push_back(std::move(pair));
  }
  return static_cast<int>(point_pairs->size()) > first_new;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
