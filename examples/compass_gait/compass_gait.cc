#include "drake/examples/compass_gait/compass_gait.h"

#include <algorithm>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace compass_gait {

template <typename T>
CompassGait<T>::CompassGait()
    : systems::LeafSystem<T>(systems::SystemTypeTag<CompassGait>{}) {
  this->DeclareContinuousState(CompassGaitContinuousState<T>(), 2, 2, 0);

  // Discrete state for stance toe distance along the ramp.
  this->DeclareDiscreteState(1);

  // Abstract state for indicating that the left foot is the stance foot.  This
  // is only used for the visualization output.
  const bool left_stance = true;
  this->DeclareAbstractState(Value<bool>(left_stance));

  // Hip torque input.
  this->DeclareVectorInputPort("hip_torque", 1);

  // The minimal state of the system.
  this->DeclareVectorOutputPort(
      "minimal_state", CompassGaitContinuousState<T>(),
      &CompassGait::MinimalStateOut, {this->all_state_ticket()});

  // The floating-base (RPY) state of the system (useful for visualization).
  this->DeclareVectorOutputPort(
      "floating_base_state", 14, &CompassGait::FloatingBaseStateOut,
      {this->all_state_ticket(), this->all_parameters_ticket()});

  this->DeclareNumericParameter(CompassGaitParams<T>());

  // Create the witness function.
  foot_collision_ = this->MakeWitnessFunction(
      "foot collision",
      systems::WitnessFunctionDirection::kPositiveThenNonPositive,
      &CompassGait::FootCollision, &CompassGait::CollisionDynamics);
}

template <typename T>
T CompassGait<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);
  const CompassGaitParams<T>& params = get_parameters(context);

  const T m = params.mass_leg();
  const T mh = params.mass_hip();
  const T l = params.length_leg();
  const T a = params.length_leg() - params.center_of_mass_leg();
  const T b = params.center_of_mass_leg();
  const T vst = cg_state.stancedot();
  const T vsw = cg_state.swingdot();

  // Sum 1/2 m*v^2 for each of the point masses.
  return .5 * (mh * l * l + m * a * a) * vst * vst +
         .5 * m * (l * l * vst * vst + b * b * vsw * vsw) -
         m * l * b * vst * vsw * cos(cg_state.swing() - cg_state.stance());
}

template <typename T>
T CompassGait<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);
  const CompassGaitParams<T>& params = get_parameters(context);

  const T m = params.mass_leg();
  const T mh = params.mass_hip();
  const T l = params.length_leg();
  const T a = params.length_leg() - params.center_of_mass_leg();
  const T b = params.center_of_mass_leg();
  const T g = params.gravity();

  const T y_toe = -get_toe_position(context) * sin(params.slope());
  const T y_hip = y_toe + l * cos(cg_state.stance());

  return m * g * (y_toe + a * cos(cg_state.stance())) + mh * g * y_hip +
         m * g * (y_hip - b * cos(cg_state.swing()));
}

template <typename T>
T CompassGait<T>::FootCollision(const systems::Context<T>& context) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);
  const CompassGaitParams<T>& params = get_parameters(context);

  // Dimensionless position of the swing foot along the ramp normal
  // (the true height scaled by 1/length_leg) is
  //    cos(stance - slope) - cos(swing - slope)
  // This is greater than zero when cos(stance - slope) > cos(swing - slope).
  // Since we know we are walking forward/downhill, we have stance - slope > 0,
  // and swing - slope < 0.  Taking acos of both sides gives that the swing foot
  // is above the ground iff stance - slope < -(swing - slope).
  // Geometrically, the intuition is that at collision then biped is forming an
  // isosceles triangle with the ramp.
  const T collision =
      2. * params.slope() - cg_state.stance() - cg_state.swing();

  // Only trigger when swing < stance (e.g. stepping forward/downhill), by
  // taking max with swing()-stance().
  //
  // Note: This is a mathematically clean way to avoid the "foot scuffing"
  // collision that is inevitable because both legs have the same length.  A
  // tempting alternative is to require some minimum step length, the foot
  // scuffing is surprisingly hard to avoid with that mechanism.
  using std::max;
  return max(collision, cg_state.swing() - cg_state.stance());
}

template <typename T>
void CompassGait<T>::CollisionDynamics(
    const systems::Context<T>& context,
    const systems::UnrestrictedUpdateEvent<T>&,
    systems::State<T>* state) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);
  CompassGaitContinuousState<T>& next_state =
      get_mutable_continuous_state(&(state->get_mutable_continuous_state()));
  const CompassGaitParams<T>& params = get_parameters(context);

  using std::cos;
  using std::sin;

  // Shorthand used in the textbook.
  const T m = params.mass_leg();
  const T mh = params.mass_hip();
  const T a = params.length_leg() - params.center_of_mass_leg();
  const T b = params.center_of_mass_leg();
  const T l = params.length_leg();
  const T cst = cos(cg_state.stance());
  const T csw = cos(cg_state.swing());
  const T hip_angle = cg_state.swing() - cg_state.stance();
  const T c = cos(hip_angle);
  const T sst = sin(cg_state.stance());
  const T ssw = sin(cg_state.swing());

  // The collision update is performed by using the minimal state and the
  // knowledge that the stance foot is on the ground to set the state of a
  // (planar) floating-base model of the biped (with the origin at the
  // pre-collision stance toe), implementing the collision, and then extracting
  // the new minimal coordinates. Mathematica derivation is here:
  // https://www.wolframcloud.com/objects/60e3d6c1-156f-4604-bf70-1e374f47140
  Matrix4<T> M_floating_base;
  // The kinematic Jacobian for the location of the pre-collision swing toe
  // about the origin (at the pre-collision stance toe).
  Eigen::Matrix<T, 2, 4> J;
  // clang-format off
  M_floating_base <<
      2 * m + mh,   0,          (m * a + m * l + mh * l) * cst,  -m * b * csw,
      0,            2 * m + mh, -(m * a + m * l + mh * l) * sst, m * b * ssw,
      (m * a + m * l + mh * l) * cst, -(m * a + m * l + mh * l) * sst,
                                m * a * a + (m + mh) * l * l,    -m * l * b * c,
      -m * b * csw, m * b * ssw, -m * l * b * c,                 m * b * b;
  J << 1, 0, l * cst,  -l * csw,
       0, 1, -l * sst, l * ssw;
  // clang-format on

  // Floating-base velocity before contact.
  Vector4<T> v_pre;
  v_pre << 0, 0, cg_state.stancedot(), cg_state.swingdot();

  const Matrix4<T> Minv = M_floating_base.inverse();
  // Floating-base velocity after contact.
  Vector4<T> v_post = v_pre - Minv * J.transpose() *
                                  (J * Minv * J.transpose()).inverse() * J *
                                  v_pre;

  // Also switch stance and swing legs:
  next_state.set_stance(cg_state.swing());
  next_state.set_swing(cg_state.stance());
  next_state.set_stancedot(v_post(3));
  next_state.set_swingdot(v_post(2));

  // toe += step length.
  set_toe_position(get_toe_position(context) -
                       2. * params.length_leg() * sin(hip_angle / 2.),
                   state);

  // Switch stance foot from left to right (or back).
  set_left_leg_is_stance(!left_leg_is_stance(context), state);
}

template <typename T>
void CompassGait<T>::MinimalStateOut(
    const systems::Context<T>& context,
    CompassGaitContinuousState<T>* output) const {
  output->SetFromVector(get_continuous_state(context).CopyToVector());
}

template <typename T>
void CompassGait<T>::FloatingBaseStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);
  const CompassGaitParams<T>& params = get_parameters(context);
  const T toe = get_toe_position(context);
  const bool left_stance = left_leg_is_stance(context);

  // x, y, z.
  output->SetAtIndex(0, toe * cos(params.slope()) +
                            params.length_leg() * sin(cg_state.stance()));
  output->SetAtIndex(1, 0.);
  output->SetAtIndex(2, -toe * sin(params.slope()) +
                            params.length_leg() * cos(cg_state.stance()));

  const T left = left_stance ? cg_state.stance() : cg_state.swing();
  const T right = left_stance ? cg_state.swing() : cg_state.stance();

  // roll, pitch, yaw.
  output->SetAtIndex(3, 0.);
  // Left leg is attached to the floating base.
  output->SetAtIndex(4, left);
  output->SetAtIndex(5, 0.);

  // Hip angle (right angle - left_angle).
  output->SetAtIndex(6, right - left);

  // x, y, z derivatives.
  output->SetAtIndex(
      7, cg_state.stancedot() * params.length_leg() * cos(cg_state.stance()));
  output->SetAtIndex(8, 0.);
  output->SetAtIndex(
      9, -cg_state.stancedot() * params.length_leg() * sin(cg_state.stance()));

  const T leftdot = left_stance ? cg_state.stancedot() : cg_state.swingdot();
  const T rightdot = left_stance ? cg_state.swingdot() : cg_state.stancedot();

  // roll, pitch, yaw derivatives.
  output->SetAtIndex(10, 0.);
  output->SetAtIndex(11, leftdot);
  output->SetAtIndex(12, 0.);

  // Hip angle derivative.
  output->SetAtIndex(13, rightdot - leftdot);
}

template <typename T>
Vector2<T> CompassGait<T>::DynamicsBiasTerm(
    const systems::Context<T>& context) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);
  const CompassGaitParams<T>& params = get_parameters(context);

  using std::sin;

  // Shorthand used in the textbook.
  const T m = params.mass_leg();
  const T mh = params.mass_hip();
  const T a = params.length_leg() - params.center_of_mass_leg();
  const T b = params.center_of_mass_leg();
  const T l = params.length_leg();
  const T g = params.gravity();
  const T s = sin(cg_state.stance() - cg_state.swing());
  const T vst = cg_state.stancedot();
  const T vsw = cg_state.swingdot();

  Vector2<T> bias{
      -m * l * b * vsw * vsw * s -
          (mh * l + m * (a + l)) * g * sin(cg_state.stance()),
      m * l * b * vst * vst * s + m * b * g * sin(cg_state.swing())};

  return bias;
}

template <typename T>
Matrix2<T> CompassGait<T>::MassMatrix(
    const systems::Context<T>& context) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);
  const CompassGaitParams<T>& params = get_parameters(context);

  using std::cos;

  // Shorthand used in the textbook.
  const T m = params.mass_leg();
  const T mh = params.mass_hip();
  const T a = params.length_leg() - params.center_of_mass_leg();
  const T b = params.center_of_mass_leg();
  const T l = params.length_leg();
  const T c = cos(cg_state.swing() - cg_state.stance());

  Matrix2<T> M;
  // clang-format off
  M << mh * l * l + m * (l * l + a * a), -m * l * b * c,
       -m * l * b * c,                   m * b * b;
  // clang-format on

  return M;
}

template <typename T>
void CompassGait<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const CompassGaitContinuousState<T>& cg_state = get_continuous_state(context);

  const Matrix2<T> M = MassMatrix(context);
  const Vector2<T> bias = DynamicsBiasTerm(context);
  const Vector2<T> B(-1, 1);
  const Vector1<T> u = this->get_input_port(0).Eval(context);

  Vector4<T> xdot;
  // clang-format off
  xdot << cg_state.stancedot(),
          cg_state.swingdot(),
          M.inverse() * (B*u - bias);
  // clang-format on
  derivatives->SetFromVector(xdot);
}

template <typename T>
void CompassGait<T>::DoGetWitnessFunctions(
    const systems::Context<T>&,
    std::vector<const systems::WitnessFunction<T>*>* witnesses) const {
  witnesses->push_back(foot_collision_.get());
}

}  // namespace compass_gait
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::compass_gait::CompassGait)
