#include "gtest/gtest.h"

#include "drake/util/testUtil.h"
#include "collision_tests.h"

using Eigen::Dynamic;

namespace Drake {
namespace {
const double epsilon = 1.0e-6;

void runBouncingBallOnInclinedPlane()
{
//This is the analytical solution at t=0.6
static const double z_analytical = -0.886544828396101;

//However, the numerical solution uses a spring model for contact forces
//and converges to slightly different number.
//Convergence is assessed in two different axes:
//  * in time: time step is sistematically reduced for a constant penetration_stiffness
//  * in penetration_stiffness: time step is kept constant and the penetration_stiffness is sistematically increased
//
//For convergence in penetration_stiffness only small enough time steps are used so that the system is stable.
//These results are summarized in the file "convergence_test"

//For penetration_stiffness = 50000.0 and delt=1.0e-3 the solution at t=0.6 is:
static const double z_precomputed = -0.88768033488430764;

collisionTests ball_test;

//compute solution
ball_test.set_time_step(1.00e-3);
ball_test.run();
double z_numerical = ball_test.get_solution();

std::cout << "Numerical sol: " << z_numerical << std::endl;

EXPECT_NO_THROW(valuecheck(z_numerical, z_precomputed, epsilon))
<< "expected: " << z_analytical << " actual " << z_numerical
<< std::endl;
}

// Validate the solution of a sphere bouncing on an inclined plane against an analytical solution.
// Descpription of the problem:
//     A sphere of diameter D=1.0 and mass m=1.0 initialliy located at x=(0,0,0) is left to free fall.
//     An inclined plane at 45 degrees angle is located underneath the sphere such that the first impact
//     occurs when the ball's center is at x=(0,0,0.5).
//     The plane inclination is positive 45 degress around the y-axis. Gravity acts in the negative z-axis with acceleration g=9.81.
//
// Analytical solution:
//     By energy conservation the magnitude of this velocity will be equal to its magnitude right before collision.
//     Since the ball falls for a height h=0.5 its velocity right before impact is vi=\sqrt{2 g h}.
//     The time it takes until the first collision therefore is ti=vi/g=0.31928
//     Since the problem is symmetric, right after collision with the inclined plane the ball will have a velocity in the positive x direction.
//     With the new horizontal velocity after the impact the ball starts free falling and its position and velocity at a time after collision
//     can be determined analytically:
//     The equation of the parabolla described by the ball in free fall is:
//                  z(t) = z0-0.5*g*t^2
//                  x(t) = vi*t
//     The equation of the inclined plane at the ball's x locaiton is:
//                  zp(t) = z0 - x(t)
//     Equating the two elevations z(t) = zp(t) allows to obtain the time it takes until the second impact.
//     This time is ti2 = 0.95783
//
//     An intermediate time between the first and second impacts is chosen: t=0.6
//     At this time z=z=z0-0.5*g*(t0-ti).^2 with z0 the position at impact (z0=-0.5).
//     Therefore: z(t=0.6) = -0.886544828396101
TEST(simple_collisions_gtest, bouncing_ball_on_inclined_plane) {
runBouncingBallOnInclinedPlane();
}

}  // namespace
}  // namespace Drake
