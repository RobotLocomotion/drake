#include "drake/multibody/fem/mpm-dev/particles_to_bgeo.h"

#include <Partio.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

void WriteParticlesToBgeo(const std::string& filename,
                          const std::vector<Vector3<double>>& q,
                          const std::vector<Vector3<double>>& v,
                          const std::vector<double>& m) {
  DRAKE_DEMAND(q.size() == v.size());
  DRAKE_DEMAND(q.size() == m.size());
  // Create a particle data handle.
  Partio::ParticlesDataMutable* particles = Partio::create();
  Partio::ParticleAttribute position;
  Partio::ParticleAttribute velocity;
  Partio::ParticleAttribute mass;
  position = particles->addAttribute("position", Partio::VECTOR, 3);
  velocity = particles->addAttribute("velocity", Partio::VECTOR, 3);
  mass = particles->addAttribute("mass", Partio::VECTOR, 1);
  for (size_t i = 0; i < q.size(); ++i) {
    int index = particles->addParticle();
    // N.B. PARTIO doesn't support double!
    float* q_dest = particles->dataWrite<float>(position, index);
    float* v_dest = particles->dataWrite<float>(velocity, index);
    float* m_dest = particles->dataWrite<float>(mass, index);
    m_dest[0] = m[i];
    for (int d = 0; d < 3; ++d) {
      q_dest[d] = q[i](d);
      v_dest[d] = v[i](d);
    }
  }
  Partio::write(filename.c_str(), *particles);
  particles->release();
}

void ReadParticlesFromBgeo(const std::string& filename,
                           std::vector<Vector3<double>>* q,
                           std::vector<Vector3<double>>* v,
                           std::vector<double>* m) {
  DRAKE_DEMAND(q != nullptr);
  DRAKE_DEMAND(v != nullptr);
  DRAKE_DEMAND(m != nullptr);
  q->clear();
  v->clear();
  m->clear();
  // TODO(xuchenhan-tri): Should probably use relative path.
  const std::string path = filename;
  // Read in the data from file.
  Partio::ParticlesData* data = Partio::read(path.c_str());
  DRAKE_THROW_UNLESS(data != nullptr);
  // Get position attributes as vector3.
  Partio::ParticleAttribute position;
  DRAKE_THROW_UNLESS(data->attributeInfo("position", position));
  DRAKE_THROW_UNLESS(position.type == Partio::VECTOR);
  DRAKE_THROW_UNLESS(position.count == 3);
  // Get velocity attributes as vector3.
  Partio::ParticleAttribute velocity;
  DRAKE_THROW_UNLESS(data->attributeInfo("velocity", velocity));
  DRAKE_THROW_UNLESS(velocity.type == Partio::VECTOR);
  DRAKE_THROW_UNLESS(velocity.count == 3);
  // Get mass attributes as vector1.
  Partio::ParticleAttribute mass;
  DRAKE_THROW_UNLESS(data->attributeInfo("mass", mass));
  DRAKE_THROW_UNLESS(mass.type == Partio::VECTOR);
  DRAKE_THROW_UNLESS(mass.count == 1);
  // Get accessors for each attribute.
  Partio::ParticleAccessor position_accessor(position);
  Partio::ParticleAccessor velocity_accessor(velocity);
  Partio::ParticleAccessor mass_accessor(mass);

  Partio::ParticlesData::const_iterator it = data->begin();
  it.addAccessor(position_accessor);
  it.addAccessor(velocity_accessor);
  it.addAccessor(mass_accessor);
  for (; it != data->end(); ++it) {
    float* q_src = position_accessor.raw<float>(it);
    float* v_src = velocity_accessor.raw<float>(it);
    float* m_src = mass_accessor.raw<float>(it);
    Vector3<double> q_dest, v_dest;
    for (size_t d = 0; d < 3; ++d) {
      q_dest(d) = q_src[d];
      v_dest(d) = v_src[d];
    }
    double m_dest = m_src[0];
    // TODO(xuchenhan-tri): Preallocate.
    q->emplace_back(q_dest);
    v->emplace_back(v_dest);
    m->emplace_back(m_dest);
  }
  data->release();
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
