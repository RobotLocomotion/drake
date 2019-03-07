#include "drake/automotive/maliput/utility/mesh_simplification.h"

#include <gtest/gtest.h>

namespace drake {
namespace maliput {
namespace utility {
namespace mesh {
namespace {

// Tests equality and inequality operator overloads
// for DirectedEdgeIndex instances.
GTEST_TEST(DirectedEdgeIndexTest, Equality) {
  const DirectedEdgeIndex edge{0, 10};
  const DirectedEdgeIndex same_end_edge{4, 10};
  const DirectedEdgeIndex same_start_edge{0, 4};
  const DirectedEdgeIndex reversed_edge{10, 0};

  EXPECT_EQ(edge, edge);
  EXPECT_NE(edge, same_end_edge);
  EXPECT_NE(edge, same_start_edge);
  EXPECT_NE(edge, reversed_edge);
}


// Tests equality and inequality operator overloads
// for FaceEdgeIndex instances.
GTEST_TEST(FaceEdgeIndexTest, Equality) {
  const FaceEdgeIndex edge{0, 10};
  const FaceEdgeIndex equivalent_edge{0, 10};
  const FaceEdgeIndex edge_on_another_face{3, 10};
  const FaceEdgeIndex other_edge_on_same_face{0, 5};

  EXPECT_EQ(edge, edge);
  EXPECT_EQ(edge, equivalent_edge);
  EXPECT_NE(edge, edge_on_another_face);
  EXPECT_NE(edge, other_edge_on_same_face);
}

// Fixture for mesh simplification tests.
//
// The mesh under test is depicted below:
//
// <pre>
//
//  I(0,2,1) ô---------------ǒ H(2,2,-1)
//           |               |
//           |               |
//           |   C(1,1,0)    | F(2,1,0)
//  D(0,1,0) o-------o-------o-------ô
//           |       |       |     /  G(3,1,1)
//           |       |       |   /
//           |       |       | /           y ^   ⊙ z
//  A(0,0,0) o-------o-------o E(2,0,0)      |
//               B(1,0,0)                    +--->
//                                               x
// </pre>
//
// Notation being used is as follows:
// - 'o' refers to a vertex on the z = 0 plane,
// - 'ô' refers to a vertex on the z = 1 plane,
// - 'ǒ' refers to a vertex on the z = -1 plane,
// - '-', '|' and '/' refer to edges.
class GeoMeshSimplificationTest : public ::testing::Test {
 protected:
  // Default constructor, initializing the mesh and its
  // simplified counterpart (simplified by merging coplanar faces
  // and removing redundant vertices).
  GeoMeshSimplificationTest() {
    const GeoNormal kNormal(api::GeoPosition::FromXyz(kNormalVector));
    const GeoVertex kVertexA(api::GeoPosition::FromXyz(kVertexAPosition));
    const GeoVertex kVertexB(api::GeoPosition::FromXyz(kVertexBPosition));
    const GeoVertex kVertexC(api::GeoPosition::FromXyz(kVertexCPosition));
    const GeoVertex kVertexD(api::GeoPosition::FromXyz(kVertexDPosition));
    const GeoVertex kVertexE(api::GeoPosition::FromXyz(kVertexEPosition));
    const GeoVertex kVertexF(api::GeoPosition::FromXyz(kVertexFPosition));
    const GeoVertex kVertexG(api::GeoPosition::FromXyz(kVertexGPosition));
    const GeoVertex kVertexH(api::GeoPosition::FromXyz(kVertexHPosition));
    const GeoVertex kVertexI(api::GeoPosition::FromXyz(kVertexIPosition));

    GeoFace first_quad_face;
    first_quad_face.push_vn(kVertexA, kNormal);
    first_quad_face.push_vn(kVertexB, kNormal);
    first_quad_face.push_vn(kVertexC, kNormal);
    first_quad_face.push_vn(kVertexD, kNormal);
    mesh_.PushFace(first_quad_face);
    GeoFace second_quad_face;
    second_quad_face.push_vn(kVertexC, kNormal);
    second_quad_face.push_vn(kVertexB, kNormal);
    second_quad_face.push_vn(kVertexE, kNormal);
    second_quad_face.push_vn(kVertexF, kNormal);
    mesh_.PushFace(second_quad_face);
    GeoFace triangle_face;
    triangle_face.push_vn(kVertexF, kNormal);
    triangle_face.push_vn(kVertexE, kNormal);
    triangle_face.push_vn(kVertexG, kNormal);
    mesh_.PushFace(triangle_face);
    GeoFace third_quad_face;
    third_quad_face.push_vn(kVertexD, kNormal);
    third_quad_face.push_vn(kVertexC, kNormal);
    third_quad_face.push_vn(kVertexF, kNormal);
    third_quad_face.push_vn(kVertexH, kNormal);
    third_quad_face.push_vn(kVertexI, kNormal);
    mesh_.PushFace(third_quad_face);
    GeoFace first_plus_second_quad_face;
    first_plus_second_quad_face.push_vn(kVertexA, kNormal);
    first_plus_second_quad_face.push_vn(kVertexE, kNormal);
    first_plus_second_quad_face.push_vn(kVertexF, kNormal);
    first_plus_second_quad_face.push_vn(kVertexD, kNormal);
    GeoFace simple_third_quad_face;
    simple_third_quad_face.push_vn(kVertexD, kNormal);
    simple_third_quad_face.push_vn(kVertexF, kNormal);
    simple_third_quad_face.push_vn(kVertexH, kNormal);
    simple_third_quad_face.push_vn(kVertexI, kNormal);
    simplified_mesh_.PushFace(first_plus_second_quad_face);
    simplified_mesh_.PushFace(triangle_face);
    simplified_mesh_.PushFace(simple_third_quad_face);
  }

  const GeoMesh& mesh() const { return mesh_; }

  const std::vector<IndexFace>& faces() const { return mesh_.faces(); }

  const GeoMesh& simplified_mesh() const { return simplified_mesh_; }

  const int kNormalIndex{0};
  const int kVertexAIndex{0};
  const int kVertexBIndex{1};
  const int kVertexCIndex{2};
  const int kVertexDIndex{3};
  const int kVertexEIndex{4};
  const int kVertexFIndex{5};
  const int kVertexGIndex{6};
  const int kVertexHIndex{7};
  const int kVertexIIndex{8};

  const Vector3<double> kNormalVector{0., 0., 1.};
  const Vector3<double> kVertexAPosition{0., 0., 0.};
  const Vector3<double> kVertexBPosition{1., 0., 0.};
  const Vector3<double> kVertexCPosition{1., 1., 0.};
  const Vector3<double> kVertexDPosition{0., 1., 0.};
  const Vector3<double> kVertexEPosition{2., 0., 0.};
  const Vector3<double> kVertexFPosition{2., 1., 0.};
  const Vector3<double> kVertexGPosition{3., 1., 1.};
  const Vector3<double> kVertexHPosition{2., 2., -1.};
  const Vector3<double> kVertexIPosition{0., 2., 1.};

  const int kFirstQuadIndex{0};
  const int kSecondQuadIndex{1};
  const int kTriangleIndex{2};
  const int kThirdQuadIndex{3};

  const FaceEdgeIndex kFirstQuadAEdge{kFirstQuadIndex, 0};
  const FaceEdgeIndex kFirstQuadBEdge{kFirstQuadIndex, 1};
  const FaceEdgeIndex kFirstQuadCEdge{kFirstQuadIndex, 2};
  const FaceEdgeIndex kFirstQuadDEdge{kFirstQuadIndex, 3};
  const FaceEdgeIndex kSecondQuadCEdge{kSecondQuadIndex, 0};
  const FaceEdgeIndex kSecondQuadBEdge{kSecondQuadIndex, 1};
  const FaceEdgeIndex kSecondQuadEEdge{kSecondQuadIndex, 2};
  const FaceEdgeIndex kSecondQuadFEdge{kSecondQuadIndex, 3};
  const FaceEdgeIndex kTriangleFEdge{kTriangleIndex, 0};
  const FaceEdgeIndex kTriangleEEdge{kTriangleIndex, 1};
  const FaceEdgeIndex kTriangleGEdge{kTriangleIndex, 2};
  const FaceEdgeIndex kThirdQuadDEdge{kThirdQuadIndex, 0};
  const FaceEdgeIndex kThirdQuadCEdge{kThirdQuadIndex, 1};
  const FaceEdgeIndex kThirdQuadFEdge{kThirdQuadIndex, 2};
  const FaceEdgeIndex kThirdQuadHEdge{kThirdQuadIndex, 3};
  const FaceEdgeIndex kThirdQuadIEdge{kThirdQuadIndex, 4};
  const FaceEdgeIndex kNoEdge{-1, -1};

  const DirectedEdgeIndex kABEdge{kVertexAIndex, kVertexBIndex};
  const DirectedEdgeIndex kBCEdge{kVertexBIndex, kVertexCIndex};
  const DirectedEdgeIndex kCBEdge{kVertexCIndex, kVertexBIndex};
  const DirectedEdgeIndex kCDEdge{kVertexCIndex, kVertexDIndex};
  const DirectedEdgeIndex kDCEdge{kVertexDIndex, kVertexCIndex};
  const DirectedEdgeIndex kDAEdge{kVertexDIndex, kVertexAIndex};
  const DirectedEdgeIndex kBEEdge{kVertexBIndex, kVertexEIndex};
  const DirectedEdgeIndex kEFEdge{kVertexEIndex, kVertexFIndex};
  const DirectedEdgeIndex kFEEdge{kVertexFIndex, kVertexEIndex};
  const DirectedEdgeIndex kFCEdge{kVertexFIndex, kVertexCIndex};
  const DirectedEdgeIndex kCFEdge{kVertexCIndex, kVertexFIndex};
  const DirectedEdgeIndex kEGEdge{kVertexEIndex, kVertexGIndex};
  const DirectedEdgeIndex kGFEdge{kVertexGIndex, kVertexFIndex};
  const DirectedEdgeIndex kFHEdge{kVertexFIndex, kVertexHIndex};
  const DirectedEdgeIndex kHIEdge{kVertexHIndex, kVertexIIndex};
  const DirectedEdgeIndex kIDEdge{kVertexIIndex, kVertexDIndex};

  const double kAlmostExact{1e-12};

 private:
  GeoMesh mesh_;
  GeoMesh simplified_mesh_;
};


TEST_F(GeoMeshSimplificationTest, InverseFaceEdgeMapComputation) {
  InverseFaceEdgeMap map = ComputeInverseFaceEdgeMap(faces());

  // Tests first quad edges.
  EXPECT_EQ(map.count(kABEdge), 1);
  EXPECT_EQ(map.at(kABEdge), kFirstQuadAEdge);
  map.erase(kABEdge);

  EXPECT_EQ(map.count(kBCEdge), 1);
  EXPECT_EQ(map.at(kBCEdge), kFirstQuadBEdge);
  map.erase(kBCEdge);

  EXPECT_EQ(map.count(kCDEdge), 1);
  EXPECT_EQ(map.at(kCDEdge), kFirstQuadCEdge);
  map.erase(kCDEdge);

  EXPECT_EQ(map.count(kDAEdge), 1);
  EXPECT_EQ(map.at(kDAEdge), kFirstQuadDEdge);
  map.erase(kDAEdge);

  // Tests second quad edges.
  EXPECT_EQ(map.count(kCBEdge), 1);
  EXPECT_EQ(map.at(kCBEdge), kSecondQuadCEdge);
  map.erase(kCBEdge);

  EXPECT_EQ(map.count(kBEEdge), 1);
  EXPECT_EQ(map.at(kBEEdge), kSecondQuadBEdge);
  map.erase(kBEEdge);

  EXPECT_EQ(map.count(kEFEdge), 1);
  EXPECT_EQ(map.at(kEFEdge), kSecondQuadEEdge);
  map.erase(kEFEdge);

  EXPECT_EQ(map.count(kFCEdge), 1);
  EXPECT_EQ(map.at(kFCEdge), kSecondQuadFEdge);
  map.erase(kFCEdge);

  // Tests triangle edges.
  EXPECT_EQ(map.count(kFEEdge), 1);
  EXPECT_EQ(map.at(kFEEdge), kTriangleFEdge);
  map.erase(kFEEdge);

  EXPECT_EQ(map.count(kEGEdge), 1);
  EXPECT_EQ(map.at(kEGEdge), kTriangleEEdge);
  map.erase(kEGEdge);

  EXPECT_EQ(map.count(kGFEdge), 1);
  EXPECT_EQ(map.at(kGFEdge), kTriangleGEdge);
  map.erase(kGFEdge);

  // Tests third quad edges.
  EXPECT_EQ(map.count(kDCEdge), 1);
  EXPECT_EQ(map.at(kDCEdge), kThirdQuadDEdge);
  map.erase(kDCEdge);

  EXPECT_EQ(map.count(kCFEdge), 1);
  EXPECT_EQ(map.at(kCFEdge), kThirdQuadCEdge);
  map.erase(kCFEdge);

  EXPECT_EQ(map.count(kFHEdge), 1);
  EXPECT_EQ(map.at(kFHEdge), kThirdQuadFEdge);
  map.erase(kFHEdge);

  EXPECT_EQ(map.count(kHIEdge), 1);
  EXPECT_EQ(map.at(kHIEdge), kThirdQuadHEdge);
  map.erase(kHIEdge);

  EXPECT_EQ(map.count(kIDEdge), 1);
  EXPECT_EQ(map.at(kIDEdge), kThirdQuadIEdge);
  map.erase(kIDEdge);

  EXPECT_TRUE(map.empty());
}


TEST_F(GeoMeshSimplificationTest, FaceAdjacencyMapComputation) {
  const FaceAdjacencyMap map = ComputeFaceAdjacencyMap(faces());

  EXPECT_EQ(map.count(kFirstQuadIndex), 1);
  {
    const std::vector<FaceEdgeIndex>& face_edges = map.at(kFirstQuadIndex);
    EXPECT_EQ(face_edges[kFirstQuadAEdge.edge_index], kNoEdge);
    EXPECT_EQ(face_edges[kFirstQuadBEdge.edge_index], kSecondQuadCEdge);
    EXPECT_EQ(face_edges[kFirstQuadCEdge.edge_index], kThirdQuadDEdge);
    EXPECT_EQ(face_edges[kFirstQuadDEdge.edge_index], kNoEdge);
  }

  EXPECT_EQ(map.count(kSecondQuadIndex), 1);
  {
    const std::vector<FaceEdgeIndex>& face_edges = map.at(kSecondQuadIndex);
    EXPECT_EQ(face_edges[kSecondQuadCEdge.edge_index], kFirstQuadBEdge);
    EXPECT_EQ(face_edges[kSecondQuadBEdge.edge_index], kNoEdge);
    EXPECT_EQ(face_edges[kSecondQuadEEdge.edge_index], kTriangleFEdge);
    EXPECT_EQ(face_edges[kSecondQuadFEdge.edge_index], kThirdQuadCEdge);
  }

  EXPECT_EQ(map.count(kTriangleIndex), 1);
  {
    const std::vector<FaceEdgeIndex>& face_edges = map.at(kTriangleIndex);
    EXPECT_EQ(face_edges[kTriangleFEdge.edge_index], kSecondQuadEEdge);
    EXPECT_EQ(face_edges[kTriangleEEdge.edge_index], kNoEdge);
    EXPECT_EQ(face_edges[kTriangleGEdge.edge_index], kNoEdge);
  }

  EXPECT_EQ(map.count(kThirdQuadIndex), 1);
  {
    const std::vector<FaceEdgeIndex>& face_edges = map.at(kThirdQuadIndex);
    EXPECT_EQ(face_edges[kThirdQuadDEdge.edge_index], kFirstQuadCEdge);
    EXPECT_EQ(face_edges[kThirdQuadCEdge.edge_index], kSecondQuadFEdge);
    EXPECT_EQ(face_edges[kThirdQuadFEdge.edge_index], kNoEdge);
    EXPECT_EQ(face_edges[kThirdQuadHEdge.edge_index], kNoEdge);
    EXPECT_EQ(face_edges[kThirdQuadIEdge.edge_index], kNoEdge);
  }
}


TEST_F(GeoMeshSimplificationTest, FaceVertexPosition) {
  const IndexFace& first_quad = faces()[kFirstQuadIndex];
  EXPECT_EQ(GetMeshFaceVertexPosition(
      mesh(), first_quad.vertices()[0]), kVertexAPosition);
  const IndexFace& second_quad = faces()[kSecondQuadIndex];
  EXPECT_EQ(GetMeshFaceVertexPosition(
      mesh(), second_quad.vertices()[0]), kVertexCPosition);
  const IndexFace& triangle = faces()[kTriangleIndex];
  EXPECT_EQ(GetMeshFaceVertexPosition(
      mesh(), triangle.vertices()[0]), kVertexFPosition);
}


TEST_F(GeoMeshSimplificationTest, FaceVertexNormal) {
  const IndexFace& first_quad = faces()[kFirstQuadIndex];
  EXPECT_EQ(GetMeshFaceVertexNormal(
      mesh(), first_quad.vertices()[0]), kNormalVector);
  const IndexFace& second_quad = faces()[kSecondQuadIndex];
  EXPECT_EQ(GetMeshFaceVertexNormal(
      mesh(), second_quad.vertices()[0]), kNormalVector);
  const IndexFace& triangle = faces()[kTriangleIndex];
  EXPECT_EQ(GetMeshFaceVertexNormal(
      mesh(), triangle.vertices()[0]), kNormalVector);
}


TEST_F(GeoMeshSimplificationTest, VerticesOnPlane) {
  const Hyperplane3<double> planeA(kNormalVector, kVertexAPosition);

  const std::vector<IndexFace::Vertex>&
      first_quad_vertices = faces()[kFirstQuadIndex].vertices();
  EXPECT_TRUE(DoMeshVerticesLieOnPlane(mesh(), first_quad_vertices.begin(),
                                       first_quad_vertices.end(), planeA,
                                       kAlmostExact));

  const std::vector<IndexFace::Vertex>&
      second_quad_vertices = faces()[kSecondQuadIndex].vertices();
  EXPECT_TRUE(DoMeshVerticesLieOnPlane(mesh(), second_quad_vertices.begin(),
                                       second_quad_vertices.end(), planeA,
                                       kAlmostExact));

  const std::vector<IndexFace::Vertex>&
      triangle_vertices = faces()[kTriangleIndex].vertices();
  EXPECT_FALSE(DoMeshVerticesLieOnPlane(mesh(), triangle_vertices.begin(),
                                        triangle_vertices.end(), planeA,
                                        kAlmostExact));
}


TEST_F(GeoMeshSimplificationTest, CoplanarFaces) {
  const Hyperplane3<double> planeA(kNormalVector, kVertexAPosition);
  EXPECT_TRUE(IsMeshFaceCoplanarWithPlane(mesh(), faces()[kFirstQuadIndex],
                                          planeA, kAlmostExact));
  EXPECT_TRUE(IsMeshFaceCoplanarWithPlane(mesh(), faces()[kSecondQuadIndex],
                                          planeA, kAlmostExact));
  EXPECT_FALSE(IsMeshFaceCoplanarWithPlane(mesh(), faces()[kTriangleIndex],
                                           planeA, kAlmostExact));
  EXPECT_FALSE(IsMeshFaceCoplanarWithPlane(mesh(), faces()[kThirdQuadIndex],
                                           planeA, kAlmostExact));
}


TEST_F(GeoMeshSimplificationTest, PlanarFaces) {
  Hyperplane3<double> plane;
  EXPECT_TRUE(IsMeshFacePlanar(
      mesh(), faces()[kFirstQuadIndex], kAlmostExact, &plane));
  EXPECT_NEAR(plane.absDistance(kVertexCPosition), 0., kAlmostExact);
  EXPECT_FALSE(IsMeshFacePlanar(
      mesh(), faces()[kThirdQuadIndex], kAlmostExact, &plane));
  // Triangle face would be planar BUT its specified normal does not
  // match that of the plane its vertices define.
  EXPECT_FALSE(IsMeshFacePlanar(
      mesh(), faces()[kTriangleIndex], kAlmostExact, &plane));
}


TEST_F(GeoMeshSimplificationTest, FaceMerging) {
  const FaceAdjacencyMap adjacency_map =
      ComputeFaceAdjacencyMap(faces());
  std::set<int> visited_faces;
  const std::set<int> merged_faces =
      AggregateAdjacentCoplanarMeshFaces(mesh(), kFirstQuadIndex,
                                         adjacency_map, kAlmostExact,
                                         &visited_faces);
  EXPECT_EQ(merged_faces.count(kFirstQuadIndex), 1);
  EXPECT_EQ(merged_faces.count(kSecondQuadIndex), 1);
  EXPECT_EQ(merged_faces.count(kThirdQuadIndex), 0);
  EXPECT_EQ(merged_faces.count(kTriangleIndex), 0);

  // In what follows, the test relies heavily on the fact that
  // std::set keys are integers and that this container uses
  // std::less as ordering criterion by default.
  const FaceEdgeIndex outer_edge =
      FindOuterFaceEdgeIndex(merged_faces, adjacency_map);
  EXPECT_EQ(outer_edge, kFirstQuadAEdge);

  const std::vector<FaceVertexIndex> merged_face_contour =
      ComputeMeshFacesContour(merged_faces, adjacency_map);
  ASSERT_EQ(merged_face_contour.size(), 6);
  EXPECT_EQ(merged_face_contour[0].face_index, kFirstQuadIndex);
  EXPECT_EQ(merged_face_contour[0].vertex_index, 0);
  EXPECT_EQ(merged_face_contour[1].face_index, kSecondQuadIndex);
  EXPECT_EQ(merged_face_contour[1].vertex_index, 1);
  EXPECT_EQ(merged_face_contour[2].face_index, kSecondQuadIndex);
  EXPECT_EQ(merged_face_contour[2].vertex_index, 2);
  EXPECT_EQ(merged_face_contour[3].face_index, kSecondQuadIndex);
  EXPECT_EQ(merged_face_contour[3].vertex_index, 3);
  EXPECT_EQ(merged_face_contour[4].face_index, kFirstQuadIndex);
  EXPECT_EQ(merged_face_contour[4].vertex_index, 2);
  EXPECT_EQ(merged_face_contour[5].face_index, kFirstQuadIndex);
  EXPECT_EQ(merged_face_contour[5].vertex_index, 3);

  const std::vector<FaceVertexIndex> simplified_contour =
      SimplifyMeshFacesContour(mesh(), merged_face_contour, kAlmostExact);
  ASSERT_EQ(simplified_contour.size(), 4);
  EXPECT_EQ(simplified_contour[0].face_index, kFirstQuadIndex);
  EXPECT_EQ(simplified_contour[0].vertex_index, 0);
  EXPECT_EQ(simplified_contour[1].face_index, kSecondQuadIndex);
  EXPECT_EQ(simplified_contour[1].vertex_index, 2);
  EXPECT_EQ(simplified_contour[2].face_index, kSecondQuadIndex);
  EXPECT_EQ(simplified_contour[2].vertex_index, 3);
  EXPECT_EQ(simplified_contour[3].face_index, kFirstQuadIndex);
  EXPECT_EQ(simplified_contour[3].vertex_index, 3);

  const GeoFace merged_face = MergeMeshFaces(
      mesh(), merged_faces, adjacency_map, kAlmostExact);
  EXPECT_EQ(merged_face.vertices()[0].v().xyz(), kVertexAPosition);
  EXPECT_EQ(merged_face.normals()[0].n().xyz(), kNormalVector);
  EXPECT_EQ(merged_face.vertices()[1].v().xyz(), kVertexEPosition);
  EXPECT_EQ(merged_face.normals()[1].n().xyz(), kNormalVector);
  EXPECT_EQ(merged_face.vertices()[2].v().xyz(), kVertexFPosition);
  EXPECT_EQ(merged_face.normals()[2].n().xyz(), kNormalVector);
  EXPECT_EQ(merged_face.vertices()[3].v().xyz(), kVertexDPosition);
  EXPECT_EQ(merged_face.normals()[3].n().xyz(), kNormalVector);
}


TEST_F(GeoMeshSimplificationTest, MeshSimplification) {
  const std::string kDummyMaterial{"stone"};
  const int kPrecision{3};
  const api::GeoPosition kOrigin(1., 2., 3.);
  const int kZeroVertexIndexOffset{2};
  const int kZeroNormalIndexOffset{0};
  std::stringstream simplified_mesh_obj;
  SimplifyMeshFaces(mesh(), kAlmostExact).EmitObj(
      simplified_mesh_obj, kDummyMaterial, kPrecision, kOrigin,
      kZeroVertexIndexOffset, kZeroNormalIndexOffset);
  std::stringstream reference_mesh_obj;
  simplified_mesh().EmitObj(
      reference_mesh_obj, kDummyMaterial, kPrecision, kOrigin,
      kZeroVertexIndexOffset, kZeroNormalIndexOffset);
  EXPECT_EQ(simplified_mesh_obj.str(), reference_mesh_obj.str());
}

}  // namespace
}  // namespace mesh
}  // namespace utility
}  // namespace maliput
}  // namespace drake
