#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_EXPRESSION_ALL
#error Do not include this file. Use "drake/common/symbolic/expression.h".
#endif

#include <initializer_list>
#include <ostream>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt.h"
#include "drake/common/random.h"

namespace drake {
namespace symbolic {
/** Represents a symbolic environment (mapping from a variable to a value).
 *
 * This class is used when we evaluate symbolic expressions or formulas which
 * include unquantified (free) variables. Here are examples:
 *
 * \code{.cpp}
 *   const Variable var_x{"x"};
 *   const Variable var_y{"y"};
 *   const Expression x{var_x};
 *   const Expression y{var_x};
 *   const Expression e1{x + y};
 *   const Expression e2{x - y};
 *   const Formula f{e1 > e2};
 *
 *   // env maps var_x to 2.0 and var_y to 3.0
 *   const Environment env{{var_x, 2.0}, {var_y, 3.0}};
 *
 *   const double res1 = e1.Evaluate(env);  // x + y => 2.0 + 3.0 =>  5.0
 *   const double res2 = e2.Evaluate(env);  // x - y => 2.0 - 3.0 => -1.0
 *   const bool res = f.Evaluate(env);  // x + y > x - y => 5.0 >= -1.0 => True
 * \endcode
 */
class Environment {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Environment);

  typedef Variable key_type;
  typedef double mapped_type;
  typedef typename std::unordered_map<key_type, mapped_type> map;
  /** std::pair<key_type, mapped_type> */
  typedef typename map::value_type value_type;
  typedef typename map::iterator iterator;
  typedef typename map::const_iterator const_iterator;

  /** Default constructor. */
  Environment() = default;

  /** List constructor. Constructs an environment from a list of (Variable *
   * double).
   *
   * @throws std::exception if @p init include a dummy variable or a NaN
   * value.
   */
  Environment(std::initializer_list<value_type> init);

  /** List constructor. Constructs an environment from a list of
   * Variable. Initializes the variables with 0.0.
   *
   * @throws std::exception if @p vars include a dummy variable.
   */
  Environment(std::initializer_list<key_type> vars);

  /** Constructs an environment from @p m (of `map` type, which is
   * `std::unordered_map`).
   *
   * @throws std::exception if @p m include a dummy variable or a NaN value.
   */
  explicit Environment(map m);

  ~Environment();

  /** Returns an iterator to the beginning. */
  iterator begin() { return map_.begin(); }
  /** Returns an iterator to the end. */
  iterator end() { return map_.end(); }
  /** Returns a const iterator to the beginning. */
  [[nodiscard]] const_iterator begin() const { return map_.cbegin(); }
  /** Returns a const iterator to the end. */
  [[nodiscard]] const_iterator end() const { return map_.cend(); }
  /** Returns a const iterator to the beginning. */
  [[nodiscard]] const_iterator cbegin() const { return map_.cbegin(); }
  /** Returns a const iterator to the end. */
  [[nodiscard]] const_iterator cend() const { return map_.cend(); }

  /** Inserts a pair (@p key, @p elem) if this environment doesn't contain @p
   * key. Similar to insert function in map, if the key already
   * exists in this environment, then calling insert(key, elem) doesn't change
   * the existing key-value in this environment.
   */
  void insert(const key_type& key, const mapped_type& elem);

  /** Given a matrix of symbolic variables @p keys and a matrix of values @p
   * elements, inserts each pair (keys(i, j), elements(i, j)) into the
   * environment if this environment doesn't contain keys(i, j) . Similar to
   * insert function in map, if keys(i, j) already exists in this
   * environment, then this function doesn't change the its existing value in
   * this environment.
   *
   * @throws std::exception if the size of @p keys is different from the size
   * of @p elements.
   */
  void insert(const Eigen::Ref<const MatrixX<key_type>>& keys,
              const Eigen::Ref<const MatrixX<mapped_type>>& elements);

  /** Checks whether the container is empty.  */
  [[nodiscard]] bool empty() const { return map_.empty(); }
  /** Returns the number of elements. */
  [[nodiscard]] size_t size() const { return map_.size(); }

  /** Finds element with specific key. */
  iterator find(const key_type& key) { return map_.find(key); }
  /** Finds element with specific key. */
  [[nodiscard]] const_iterator find(const key_type& key) const {
    return map_.find(key);
  }

  /** Returns the domain of this environment. */
  [[nodiscard]] Variables domain() const;

  /** Returns string representation. */
  [[nodiscard]] std::string to_string() const;

  /** Returns a reference to the value that is mapped to a key equivalent to
   *  @p key, performing an insertion if such key does not already exist.
   */
  mapped_type& operator[](const key_type& key);

  /** As above, but returns a constref and does not perform an insertion
   * (throwing a runtime error instead) if the key does not exist. */
  const mapped_type& operator[](const key_type& key) const;

  friend std::ostream& operator<<(std::ostream& os, const Environment& env);

 private:
  map map_;
};

/** Populates the environment @p env by sampling values for the unassigned
 *  random variables in @p variables using @p random_generator. */
Environment PopulateRandomVariables(Environment env, const Variables& variables,
                                    RandomGenerator* random_generator);

}  // namespace symbolic
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::symbolic, Environment, env, env.to_string())
