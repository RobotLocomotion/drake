#pragma once

#include <optional>
#include <string>

#include <curl/curl.h>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

struct RenderClientParams {
  std::optional<RenderLabel> default_label{};
  std::string url{"http://127.0.0.1"};
  unsigned port{8000};
  std::string upload_endpoint{"upload"};
  std::string render_endpoint{"render"};
  bool curl_verbose = true;
};

class RenderClient : virtual public RenderEngine {
 public:
  /** \name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderClient(const RenderClient&) = delete;
#endif
  RenderClient& operator=(const RenderClient&) = delete;
  RenderClient(RenderClient&&) = delete;
  RenderClient& operator=(RenderClient&&) = delete;
  //@}}

  virtual ~RenderClient();

  /** Constructs the render engine from the given `parameters`.  */
  explicit RenderClient(
      const RenderClientParams& parameters = RenderClientParams());

  /* TODO(svenevs): prototype for discussion, RenderClient can be recovered
   whereas RenderClientGLTF cannot be (via vtk header exclude rule).  Users can
   dynamic_cast from RenderEngine, enabling us to expose customization points.
   */
  std::string ProofOfConcept() const { return "ProofOfConcept"; }

 protected:
  // TODO(svenevs): why can't this be used?  (Diamond...)
  // Allow derived classes to implement Cloning via copy-construction.
  // DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderClient)
  RenderClient(const RenderClient& other);

  std::string temp_directory_;
  std::string url_;
  unsigned port_;
  std::string upload_endpoint_;
  std::string render_endpoint_;
  bool curl_verbose_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
