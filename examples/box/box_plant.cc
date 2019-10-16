#include "drake/examples/box/box_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"
#include <fstream>
#include <type_traits>

namespace drake {
namespace examples {
namespace box {

template <typename T>
BoxPlant<T>::BoxPlant(double i_m, double l, double d, double f_n,
    double mu_s, double mu_k, double v_s)
    :  systems::VectorSystem<T>(systems::SystemTypeTag<box::BoxPlant>{},1 /* input size */, 2 /* output size */),
    i_m_(i_m) /* inverse mass */, l_(l) /* length */, d_(d) /* velocity damping */, f_n_(f_n) /* normal force */,
    mu_s_(mu_s) /* stiction coeff */, mu_k_(mu_k) /* kinetic friction coeff */, v_s_(v_s) /* stiction tolerance */
{
  DRAKE_DEMAND(v_s >= 0.0);
  DRAKE_DEMAND(f_n >= 0.0);
  this->DeclareContinuousState(drake::systems::BasicVector<T>(2) /* state size */, 1 /* num_q */, 1 /* num_v */,
                               0 /* num_z */);
}
template <typename T>
BoxPlant<T>::BoxPlant(double i_m, double l, double d) : BoxPlant(i_m, l, d, 0.0 /* normal force */,
  0. /* mu_s */, 0. /* mu_k */, 0.01 /* v_s */) {
    if (i_m > 0)
    {
      f_n_ = 9.8 / i_m;
    }
}


template <typename T>
BoxPlant<T>::BoxPlant() : BoxPlant(1.0 /* inv mass */, 
    1.0 /* length */, 0.0 /* vel damp */) {

}

template <typename T>
template <typename U>
BoxPlant<T>::BoxPlant(const BoxPlant<U>& other) : BoxPlant(other.i_m_, other.l_,
   other.d_, other.f_n_, other.mu_s_, other.mu_k_, other.v_s_) {}

template <typename T>
BoxPlant<T>::~BoxPlant() = default;

template <typename T>
void BoxPlant<T>::DoCalcVectorOutput(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const 
      {
        unused(input);
        unused(context);
        *output = state;
      }

template <typename T>
T BoxPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const VectorX<T>& state = this->GetVectorState(context);
  // Kinetic energy = 1/2 m q-dot^2
  T kinetic_energy = 0.;
  if (i_m_ != 0. )
      kinetic_energy += 0.5  * pow(state(1), 2) / i_m_;
  // no spring, so no potential energy
  return kinetic_energy;
}

template <typename T>
T BoxPlant<T>::CalcFrictionFromVelocity(T velocity) const {
  using std::sqrt;
  using std::min;
  using std::max;
  double rel_tolerance = 0.01; /* from implicit stribeck solver code */
  double eps = rel_tolerance * v_s_;
  T v_t = velocity;
  T v_t_eps = sqrt( v_t * v_t + eps * eps ) ;
  T x = v_t_eps / v_s_;
  T mu = mu_s_ * max(min(x, 1.0), x * (2.0 - x));
  return - mu * v_t / v_t_eps * f_n_;
}

template <typename T>
T BoxPlant<T>::CalcFriction(const systems::Context<T>& context) const {
  const VectorX<T>& state = this->GetVectorState(context);
  return BoxPlant<T>::CalcFrictionFromVelocity(state[1]);
}

template <typename T>
void BoxPlant<T>::set_initial_state(
    systems::Context<T>* context, const Eigen::Ref< const VectorX<T> >& z0) {
  systems::VectorBase<T>& state_vector = context->get_mutable_continuous_state_vector();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_ASSERT(z0.rows() == state_vector.size() && z0.cols() == 1);
  state_vector.SetFromVector(z0);
}

template <>
void BoxPlant<double>::StoreEigenCSVwFriction(const std::string& filename, const VectorX<double>& times, const MatrixX<double>& data, const VectorX<double>& input_vector) const {
  /* csv format from  https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format */
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                  Eigen::DontAlignCols, ", ", "\n");
  DRAKE_DEMAND(times.rows() == input_vector.rows());
  DRAKE_DEMAND(times.rows() == data.cols());
  std::ofstream file(filename);
  file << "t, input u, box_x, box_v, friction, timestep size" << std::endl;
  VectorX<double> friction(data.cols());
  for( int i = 0; i < data.cols(); i++)
  {
    double velocity = data(1,i);
    friction(i) = CalcFrictionFromVelocity(velocity);
  }

  VectorX<double> tssize(times.rows());
  tssize << times.tail(times.rows()-1) - times.head(times.rows()-1), 0.0;
  
  /* horizontally concatenate times and data */
  MatrixX<double> OutMatrix(times.rows(), times.cols() + input_vector.cols()
                            + data.rows() + friction.cols() + tssize.cols());
  OutMatrix << times, input_vector, data.transpose(), friction, tssize; /* can also do this with blocks */
  file << OutMatrix.format(CSVFormat);
  file.close();
}


template <typename T>
void BoxPlant<T>::StoreEigenCSVwFriction(const std::string&, const VectorX<double>& , const MatrixX<double>&, const VectorX<double>& ) const {
  throw new std::runtime_error("CSV Output is only supported for BoxPlant<double>.");
}

// Compute the actual physics.
template <typename T>
void BoxPlant<T>::DoCalcVectorTimeDerivatives(const systems::Context< T > &context, 
      const Eigen::VectorBlock< const VectorX< T >> &input, 
      const Eigen::VectorBlock< const VectorX< T >> &state, 
      Eigen::VectorBlock< VectorX< T >> *derivatives) const  {
  unused(context);
  (*derivatives)[0] = state[1];
  (*derivatives)[1] = (input[0] /* force */ - d_ * state[1] /* damping */ + CalcFrictionFromVelocity(state[1])) * i_m_ /* inv mass */ ;
}

static int outcount = 0;
template <typename T>
double BoxPlant<T>::CalcIterationLimiterAlpha(const VectorX<T>& x0, const VectorX<T>& dx) const
{
  double alpha = 1.;
  double init_vel = ExtractDoubleOrThrow( x0(1));
  double result_vel = ExtractDoubleOrThrow( x0(1) + dx(1) );
  using std::abs;
  double dxsignedvel = ExtractDoubleOrThrow( dx(1));
  double dxvel = abs(dxsignedvel);
  // FROM SLIDING TO AT LEAST OPPOSITE ALMOST-SLIDING
  double sliding_thres = 0.85;
  double eps = 0.01 * v_s_;
  if( ( abs(init_vel) >  sliding_thres * v_s_ ) && ( result_vel * init_vel < 0. ) && ( abs(result_vel) > sliding_thres * v_s_) )
  {
    
    alpha = (abs( init_vel) - 0.5 * v_s_  ) / dxvel;
    std::cout << "Sliding to opposite! alpha = " << alpha << " steps since last:" << outcount << std::endl;
    outcount = 0;
  }

  // FROM SLIDING TO SOFTNORM REGION
  
  else if( ( abs(init_vel) >  sliding_thres * v_s_ ) && ( abs(result_vel) < eps )  )
  {
    alpha = (abs( init_vel) - 0.5 * v_s_  ) / dxvel;
    std::cout << "Sliding to softnorm! alpha = " << alpha << " steps since last:" << outcount << std::endl;
    outcount = 0;
  }

  // FROM SOFTNORM REGION TO SLIDING
  else if( ( abs(init_vel) <  eps ) && ( abs(result_vel) > sliding_thres * v_s_ )  )
  {
    alpha = ( 0.5 * v_s_  ) / dxvel;
    std::cout << "Softnorm to sliding! alpha = " << alpha << " steps since last:" << outcount << std::endl;
    outcount = 0;
  }

  // FROM STICTION TO SLIDING -- ONLY FOR EXPLICIT -- THIS IS A HACK THAT WE WILL NOT USE
/*   else if( ( abs(init_vel) <  sliding_thres * v_s_ ) && ( abs(result_vel) > 2 * sliding_thres * v_s_) )
  {
    alpha = (result_vel *  v_s_ * 1.05 * sliding_thres / abs(result_vel)  - init_vel ) / dxsignedvel;
    std::cout << "Stiction to sliding! alpha = " << alpha << " steps since last:" << outcount << std::endl;
    outcount = 0;
  } */

  if( alpha < 0. || alpha > 1.)
  {
    std::cout << "init vel: " << init_vel << ", result_vel: " << result_vel << std::endl;
    std::cout << "alpha: " << alpha << std::endl;
    throw std::runtime_error("Error: alpha is negative or larger than 1 ");
  }
  outcount++;
  //std::cout << "alpha: " << alpha << std::endl;
  //std::cout << "\nx0:\n" << x0 << std::endl;
  //std::cout << "\ndx:\n" << dx << std::endl;
  return alpha;

}


}  // namespace box
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::box::BoxPlant)
