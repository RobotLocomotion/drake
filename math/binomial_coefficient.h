#pragma once

namespace drake {
namespace math {

/** Computes the binomial coefficient `n`-choose-`k` efficiently using a dynamic
 programming recursion.  https://en.wikipedia.org/wiki/Binomial_coefficient

 @pre k >= 0
 */
int BinomialCoefficient(int n, int k);

}  // namespace math
}  // namespace drake
