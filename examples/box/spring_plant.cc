#include "drake/examples/box/spring_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace box {

template <typename T>
SpringPlant<T>::SpringPlant()
    :  systems::LeafSystem<T>(systems::SystemTypeTag<box::SpringPlant>{}) {
 
  this->DeclareVectorInputPort("box1state",systems::BasicVector<T>(2));
  this->DeclareVectorInputPort("box2state",systems::BasicVector<T>(2));
  this->DeclareVectorOutputPort("forces",systems::BasicVector<T>(1),&SpringPlant::CalcVectorOutput);
  /* params are stored in the class */
}


template <typename T>
SpringPlant<T>::SpringPlant(double k, double d, double l)
    :  SpringPlant() {
  k_ = k;
  d_ = d;
  l_ = l;
}

template <typename T>
template <typename U>
SpringPlant<T>::SpringPlant(const SpringPlant<U>& other) : SpringPlant(other.k_, other.d_, other.l_) {}

template <typename T>
SpringPlant<T>::~SpringPlant() = default;

template <typename T>
void SpringPlant<T>::CalcVectorOutput(
      const systems::Context<T>& context,
      systems::BasicVector<T>* output) const 
      {
        const VectorX<T>& box1 = this->get_first_box_input_port().Eval(context);
        const VectorX<T>& box2 = this->get_second_box_input_port().Eval(context);
        T qdiff = box2(0) - box1(0);
        T vdiff = box2(1) - box1(1);
        using std::max;
        using std::abs;
        T xp = max( l_ - abs(qdiff), 0.);
        double eps = 1e-13;
        T sign = qdiff / (abs(qdiff) + eps);
        T xdot = - sign * vdiff ;
        T k = k_ * max( 1. + d_ * xdot, 0.);
        T f =  sign * k * xp ;
        //std::cout << qdiff << " " << vdiff << " " << f << std::endl;
        /* penalty force model */
        (*output)[0] = f;
      }
template <typename T>
const systems::OutputPort<T>& SpringPlant<T>::get_force_output_port() const {
    DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
    return systems::LeafSystem<T>::get_output_port(0);
  }

template <typename T>
const systems::InputPort<T>& SpringPlant<T>::get_first_box_input_port() const {
    DRAKE_DEMAND(systems::LeafSystem<T>::num_input_ports() == 2);
    return systems::LeafSystem<T>::get_input_port(0);
  }

template <typename T>
const systems::InputPort<T>& SpringPlant<T>::get_second_box_input_port() const {
    DRAKE_DEMAND(systems::LeafSystem<T>::num_input_ports() == 2);
    return systems::LeafSystem<T>::get_input_port(1);
  }

template <typename T>
T SpringPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  unused(context);
  // nothing meaningful yet
  return 0.0;
}


}  // namespace box
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::drake::examples::box::SpringPlant)
//DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(class ::drake::examples::box::SpringPlant )
