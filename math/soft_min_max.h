#pragma once

#include <vector>

namespace drake {
namespace math {
/**
 Computes a smooth over approximation of max function, namely SoftOverMax(x) >=
 max(x).
 Mathematically we compute this as (log (∑ᵢ exp(αxᵢ))) / α.
 @param x The vector for which we want to compute its soft max.
 @param alpha α in the documentation above. Larger α makes the soft max more
 similar to max, with a sharper corner. Must be strictly positive and finite.
 @default is 1.
 @throws std::exception if α <= 0.
 @throws std::exception if α is non-finite.
 @tparam_nonsymbolic_scalar */
template <typename T>
[[nodiscard]] T SoftOverMax(const std::vector<T>& x, double alpha = 1.0);

/**
 Computes a smooth under approximation of max function, namely SoftUnderMax(x)
 <= max(x). Mathematically we compute this as ∑ᵢ exp(αxᵢ)*xᵢ / d, where d = ∑ⱼ
 exp(αxⱼ)
 @param x The vector for which we want to compute its soft max.
 @param alpha α in the documentation above. Larger α makes the soft max more
 similar to max, with a sharper corner. Must be strictly positive and finite.
 @default is 1.
 @throws std::exception if α <= 0.
 @throws std::exception if α is non-finite.
 @tparam_nonsymbolic_scalar */
template <typename T>
[[nodiscard]] T SoftUnderMax(const std::vector<T>& x, double alpha = 1.0);

/**
 Computes a smooth over approximation of min function, namely SoftOverMin(x) >=
 min(x).
 Mathematically we compute this as ∑ᵢ exp(-αxᵢ)*xᵢ / d, where d = ∑ⱼ
 exp(-αxⱼ)
 @param x The vector for which we want to compute its soft min.
 @param alpha α in the documentation above. Larger α makes the soft min more
 similar to min, with a sharper corner. Must be strictly positive and finite.
 @default is 1.
 @throws std::exception if α <= 0.
 @throws std::exception if α is non-finite.
 @tparam_nonsymbolic_scalar */
template <typename T>
[[nodiscard]] T SoftOverMin(const std::vector<T>& x, double alpha = 1.0);

/**
 Computes a smooth under approximation of min function, namely SoftUnderMin(x)
 <= min(x). Mathematically we compute this as -(log (∑ᵢ exp(-αxᵢ))) / α
 @param x The vector for which we want to compute its soft min.
 @param alpha α in the documentation above. Larger α makes the soft min more
 similar to min, with a sharper corner. Must be strictly positive and finite.
 @default is 1.
 @throws std::exception if α <= 0.
 @throws std::exception if α is non-finite.
 @tparam_nonsymbolic_scalar */
template <typename T>
[[nodiscard]] T SoftUnderMin(const std::vector<T>& x, double alpha = 1.0);
}  // namespace math
}  // namespace drake
