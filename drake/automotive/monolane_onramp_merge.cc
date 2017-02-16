#include "drake/automotive/monolane_onramp_merge.h"

#include <cmath>
#include <memory>
#include <utility>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace automotive {

namespace mono = maliput::monolane;

template <typename T>
void MonolaneOnrampMerge<T>::BuildOnramp() {
  // Initialize the road from the origin.
  const mono::EndpointXy origin_xy{0., 0., 0.};
  const mono::EndpointZ flat_z{0., 0., 0., 0.};
  mono::Endpoint road_origin{origin_xy, flat_z};

  // Construct the pre-merge road.
  auto pre0 = rb_->Connect("pre0", road_origin,
                           mono::ArcOffset(25., -40. / 25.), flat_z);
  auto pre1 = rb_->Connect("pre1", pre0->end(),
                          mono::ArcOffset(25., 40. / 25.), flat_z);
  auto pre2 = rb_->Connect("pre2", pre1->end(),
                          mono::ArcOffset(25., -40. / 25.), flat_z);
  auto pre3 = rb_->Connect("pre3", pre2->end(),
                          mono::ArcOffset(25., 40. / 25.), flat_z);
  auto pre4 = rb_->Connect("pre4", pre3->end(),
                          mono::ArcOffset(25., -40. / 25.), flat_z);
  auto pre5 = rb_->Connect("pre5", pre4->end(),
                          mono::ArcOffset(25., 40. / 25.), flat_z);

  // Construct the post-merge road.
  rb_->Connect("post0", pre5->end(), 50., flat_z);

  // Construct the on-ramp (starting at merge junction and working backwards).
  auto onramp1 = rb_->Connect("onramp1", pre5->end(),
                             mono::ArcOffset(30., 50. / 30.), flat_z);
  rb_->Connect("onramp0", onramp1->end(), 100., flat_z);

  rg_ = rb_->Build({"monolane-merge-example"});
}

template class MonolaneOnrampMerge<double>;
// TODO(jadecastro): Bring instantiations online for `TaylorVarXd>` and
// `symbolic::Expression` types.

}  // namespace automotive
}  // namespace drake
