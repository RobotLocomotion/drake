#include "drake/geometry/optimization/implicit_graph_of_convex_sets.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"

namespace drake {
namespace geometry {
namespace optimization {

using Edge = GraphOfConvexSets::Edge;
using EdgeId = GraphOfConvexSets::EdgeId;
using Vertex = GraphOfConvexSets::Vertex;
using VertexId = GraphOfConvexSets::VertexId;

using Eigen::Vector2d;

/* A graph with one edge definitely on the optimal path, and one definitely off
it.
┌──────┐         ┌──────┐
│source├──e_on──►│target│
└───┬──┘         └──────┘
    │e_off
    │
┌───▼──┐
│ sink │
└──────┘
*/
class ThreePointsGcs : public ImplicitGraphOfConvexSets {
 public:
  ThreePointsGcs()
      : ImplicitGraphOfConvexSets(),
        source_{gcs()->AddVertex(Point(Vector2d(3., 5.)), "source")} {}

  Vertex* source() { return source_; }

 private:
  Vertex* source_{nullptr};

  std::vector<Edge*> DoSuccessors(Vertex* v) override {
    if (v == source_) {
      Vertex* target = gcs()->AddVertex(Point(Vector2d(-2., 4.)), "target");
      Vertex* sink = gcs()->AddVertex(Point(Vector2d(5., -2.3)), "sink");
      Edge* e_on = gcs()->AddEdge(v, target);
      Edge* e_off = gcs()->AddEdge(v, sink);
      return {e_on, e_off};
    }
    return {};
  }
};

GTEST_TEST(ImplicitGraphOfConvexSetsTest, ThreePointsBasic) {
  ThreePointsGcs implicit_gcs;
  auto edges = implicit_gcs.Successors(implicit_gcs.source());
  EXPECT_EQ(edges.size(), 2);

  const GraphOfConvexSets& explicit_gcs =
      implicit_gcs.BuildExplicitGcs(implicit_gcs.source());
  EXPECT_EQ(explicit_gcs.num_vertices(), 3);
  EXPECT_EQ(explicit_gcs.num_edges(), 2);
}

GTEST_TEST(ImplicitGraphOfConvexSetsTest, ThreePointsBadVertex) {
  ThreePointsGcs implicit_gcs;
  GraphOfConvexSets other_gcs;
  Vertex* other_vertex = other_gcs.AddVertex(Point(Vector2d(0., 0.)), "other");

  DRAKE_EXPECT_THROWS_MESSAGE(
      implicit_gcs.Successors(other_vertex),
      ".* is not associated with this implicit graph.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      implicit_gcs.BuildExplicitGcs(other_vertex),
      ".* is not associated with this implicit graph.*");
}

/* A simple loop graph, a -> b -> c -> a, where the vertices are identified
based on their string names. */
class SimpleLoopGcs : public ImplicitGraphOfConvexSets {
 public:
  SimpleLoopGcs() : ImplicitGraphOfConvexSets() {}

  /* Use a vertex cache to add the vertex only if needed. */
  Vertex* GetVertex(std::string name) {
    /* We'll use xa = 1, xb = 2, xc = 3. */
    double xv = 1.0 + (name[0] - 'a');
    if (auto it = vertex_cache_.find(name); it != vertex_cache_.end()) {
      return it->second;
    } else {
      auto [it2, success] = vertex_cache_.insert(
          {name, gcs()->AddVertex(Point(Vector1d(xv)), name)});
      DRAKE_DEMAND(success);
      return it2->second;
    }
  }

 private:
  std::vector<Edge*> DoSuccessors(Vertex* v) override {
    if (v->name() == "c") {
      return {gcs()->AddEdge(v, GetVertex("a"))};
    } else {
      std::string next_vertex = {static_cast<char>(v->name()[0] + 1)};
      return {gcs()->AddEdge(v, GetVertex(next_vertex))};
    }
  }

  std::map<std::string, Vertex*> vertex_cache_{};
};

GTEST_TEST(ImplicitGraphOfConvexSetsTest, SimpleLoopGcs) {
  SimpleLoopGcs implicit_gcs;
  const GraphOfConvexSets& explicit_gcs =
      implicit_gcs.BuildExplicitGcs(implicit_gcs.GetVertex("a"));
  EXPECT_EQ(explicit_gcs.num_vertices(), 3);
  EXPECT_EQ(explicit_gcs.num_edges(), 3);

  // We can also start from "b".
  const GraphOfConvexSets& explicit_gcs2 =
      implicit_gcs.BuildExplicitGcs(implicit_gcs.GetVertex("b"));
  EXPECT_EQ(explicit_gcs2.num_vertices(), 3);
  EXPECT_EQ(explicit_gcs2.num_edges(), 3);
}

/* A simple 2D grid defined by convex sets (with dimensions 1x1) and edges only
to the neighbors on the grid. In addition, the vertices are also "colored" by
type (h or v), where h types only have outgoing edges allowing horizontal
transitions, and v types only have outgoing edges for vertical transitions.
┌─────┐─────┐─────┐
│  h  │  v  │  h  │
└─────┘─────┘─────┘
│  v  │  h  │  v  │
└─────┘─────┘─────┘
│  h  │  v  │  h  │
└─────┘─────┘─────┘
This is intended to capture a bit of the organization that one might need to
e.g. implement a contact planning problem.
*/
class GridGcs : public ImplicitGraphOfConvexSets {
 public:
  enum class VertexType { kH, kV };
  struct VertexKey {
    int x;
    int y;
    VertexType type;

    // Define operator< for use in map
    bool operator<(const VertexKey& other) const {
      // First compare x coordinates
      if (x != other.x) {
        return x < other.x;
      }
      // If x's are equal, compare y coordinates
      if (y != other.y) {
        return y < other.y;
      }
      // If x and y are equal, compare types
      return type < other.type;
    }
  };

  GridGcs() : ImplicitGraphOfConvexSets() {}

  Vertex* GetVertex(int x, int y, VertexType type) {
    std::string name = fmt::format("({}, {})", x, y);
    VertexKey key{x, y, type};
    if (auto it = vertex_lookup_.find(key); it != vertex_lookup_.end()) {
      return it->second;
    } else {
      Vertex* v = gcs()->AddVertex(
          HPolyhedron::MakeBox(Vector2d(x, y), Vector2d(x + 1, y + 1)), name);
      auto [it2, success2] = vertex_keys_.insert({v->id(), key});
      DRAKE_DEMAND(success2);
      auto [it3, success3] = vertex_lookup_.insert({key, v});
      DRAKE_DEMAND(success3);
      return it3->second;
    }
  }

 private:
  std::vector<Edge*> DoSuccessors(Vertex* v) override {
    DRAKE_DEMAND(v != nullptr);
    auto it = vertex_keys_.find(v->id());
    DRAKE_DEMAND(it != vertex_keys_.end());
    VertexKey key = it->second;
    std::vector<Edge*> successors;
    if (key.type == VertexType::kH) {
      if (key.x < 3) {
        successors.push_back(
            gcs()->AddEdge(v, GetVertex(key.x + 1, key.y, VertexType::kV)));
      }
      if (key.x > 1) {
        successors.push_back(
            gcs()->AddEdge(v, GetVertex(key.x - 1, key.y, VertexType::kV)));
      }
    } else {
      if (key.y < 3) {
        successors.push_back(
            gcs()->AddEdge(v, GetVertex(key.x, key.y + 1, VertexType::kH)));
      }
      if (key.y > 1) {
        successors.push_back(
            gcs()->AddEdge(v, GetVertex(key.x, key.y - 1, VertexType::kH)));
      }
    }
    return successors;
  }

  std::map<VertexId, VertexKey> vertex_keys_{};
  std::map<VertexKey, Vertex*> vertex_lookup_{};
};

GTEST_TEST(ImplicitGraphOfConvexSetsTest, GridGcs) {
  GridGcs implicit_gcs;
  const GraphOfConvexSets& explicit_gcs = implicit_gcs.BuildExplicitGcs(
      implicit_gcs.GetVertex(1, 1, GridGcs::VertexType::kH));
  EXPECT_EQ(explicit_gcs.num_vertices(), 9);
  EXPECT_EQ(explicit_gcs.num_edges(), 12);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
