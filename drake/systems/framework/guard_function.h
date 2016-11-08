#pragma once

namespace drake {
namespace systems {

template <class T>
class GuardFunction {
 public:
  /// Constructs this guard function with a reference to the specified system.
  GuardFunction(const System<T>& system) : system_(system) {}

  /// Two possible types of guard function output types.
  enum GuardFunctionType { kContinuous, kDiscontinuous };

  /// Directions of interest for zero crossings.
  enum DirectionType {
    /**
     * Zero crossings of interest are for both increasing (rising) and
     * decreasing (falling) guard functions.
     */
    kRisingAndFalling,

    /**
     * Zero crossings of interest are for increasing (rising) guard functions
     * only.
     */
    kRising,

    /**
     * Zero crossings of interest are for decreasing (falling) guard functions
     * only.
     */
    kFalling
  };

  /**
   * Gets the direction of interest for this guard function. The direction
   * of interest can indicate that the guard should trigger only on positive to
   * negative transitions (kFalling), only on negative to positive transitions
   * (kRising), or any time zero is crossed (kRisingAndFalling).
   * @param index an index in the half-open interval
   *     [0, num_guard_function_outputs)
   */
  DirectionType get_guard_function_direction(int index) const = 0;

  /**
   *  Returns whether this guard function returns a continuous output or
   *  a discontinuous output. A greater collection of methods for determining
   *  zero crossings is applicable for guard functions with continuous outputs.
   * @param index an index in the half-open interval
   *     [0, num_guard_function_outputs)
   */
  virtual GuardFunctionType get_guard_function_output_type(int index) const = 0;

  /**
   * Returns the number of outputs for the guard function.
   */
  virtual int num_guard_function_outputs() const = 0;

  /**
   * Evaluates the guard function under the given context.
   * @returns a VectorX of dimension num_guard_function_outputs().
   */
  virtual VectorX<T> EvalGuard(const Context<T>& context) = 0;

  /**
   * Returns `true` if the derivative for the guard function with respect to
   * time is available and rapidly computable (returns `false` otherwise).
   * @sa EvalGuardDeriv()
   */
  virtual bool is_deriv_available() const = 0;

  /**
   * Returns the derivative for the guard function with respect to time.
   * This function only need be implemented if is_deriv_available() returns
   * `true`. This default implementation does nothing.
   * @sa is_deriv_available()
   * @returns a VectorX of dimension num_guard_function_outputs().
   */
  virtual VectorX<T> EvalGuardDeriv(const Context<T>& context) {}

 private:

  const System<T>& system_;
};
} // namespace systems
} // namespace drake