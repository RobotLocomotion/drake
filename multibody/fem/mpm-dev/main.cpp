#include <iostream>
#include <string>
#include <vector>

#include <Partio.h>

#include "drake/common/eigen_types.h"
#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/multibody/fem/mpm-dev/particles_to_bgeo.h"

// TODO(yiminlin.tri): pass in by pointer
// TODO(yiminlin.tri): inlining
// TODO(yiminlin.tri): Refactor, helper function to evaluate 1D quadratic spline
// Reference element: r \in [-1.5, 1.5]
double eval_1D_basis(double r) {
    double r_abs    = std::abs(r);
    if (r_abs >= 1.5) {
        return 0.0;
    } else if (r_abs < 1.5 && r_abs >= 0.5) {
        return .5*std::pow(1.5-r_abs, 2);
    } else {
        return 0.75-r_abs*r_abs;
    }
}

class Particle {
 public:
    // TODO(yiminlin.tri): constructor using Vector3, default constructor
    Particle(double m, double x, double y, double z,
                       double u1, double u2, double u3) {
        mass_     = m;
        position_ = drake::Vector3<double>{x, y, z};
        velocity_ = drake::Vector3<double>{u1, u2, u3};
    }

    double get_mass() {
        return mass_;
    }

    drake::Vector3<double> get_position() {
        return position_;
    }

    drake::Vector3<double> get_velocity() {
        return velocity_;
    }

    // TODO(yiminlin.tri):
    void get_mass(const int& m);
    void get_position(const drake::Vector3<double>& position);
    void get_velocity(const drake::Vector3<double>& velocity);

 private:
    double mass_{};                                   // Mass
    drake::Vector3<double> position_{};               // Coordinate
    drake::Vector3<double> velocity_{};               // Velocity
};  // class Particle

class GridPoint {
 public:
    // TODO(yiminlin.tri): constructor using Vector3, default constructor
    GridPoint() {}
    GridPoint(double m, double x, double y, double z,
                        double u1, double u2, double u3) {
        mass_     = m;
        position_ = drake::Vector3<double>{x, y, z};
        velocity_ = drake::Vector3<double>{u1, u2, u3};
    }

    double get_mass() {
        return mass_;
    }

    drake::Vector3<double> get_position() {
        return position_;
    }

    drake::Vector3<double> get_velocity() {
        return velocity_;
    }

    // TODO(yiminlin.tri):
    // shouldn't need to pass in h when put in MPMsystem class
    // pass in pointer
    // Evaluate nodal basis at x
    double eval_basis_3D(const drake::Vector3<double>& x, const double h) {
        // TODO(yiminlin.tri): allocation...
        drake::Vector3<double> xshifted = x-get_position();
        return eval_1D_basis(xshifted(0)/h)
              *eval_1D_basis(xshifted(1)/h)
              *eval_1D_basis(xshifted(2)/h);
    }

    // Evaluate nodal basis at position of the particle p
    // double eval_basis_3D(const Particle& p, double h) {
    //     return eval_basis_3D(p.get_position(), h);
    // }
 private:
    double mass_{};                                   // Mass
    drake::Vector3<double> position_{};               // Coordinate
    drake::Vector3<double> velocity_{};               // Velocity
};

// TODO(yiminlin.tri): refactor test files, use gtest
void test_eigen_functionalities() {
    drake::Vector3<double> tmpvec{1.0, 2.0, 3.0};
    drake::Vector3<double> tmpvec2{4.0, 5.0, 6.0};
    drake::Vector3<double> tmpvec3 = tmpvec2-tmpvec;
    drake::VectorX<drake::Vector3<double>> vec_arr;

    std::cout << "\n==================";
    std::cout << "\n====TEST EIGEN====";
    std::cout << "\n==================";
    std::cout << "\n====print tmpvec====\n";
    std::cout << tmpvec(0) << std::endl;
    std::cout << tmpvec(1) << std::endl;
    std::cout << tmpvec(2) << std::endl;
    std::cout << tmpvec3(0) << std::endl;
    std::cout << tmpvec3(1) << std::endl;
    std::cout << tmpvec3(2) << std::endl;
}

void test_class_particle() {
    Particle p1 = Particle(10.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    std::cout << "\n=====================";
    std::cout << "\n====TEST PARTICLE====";
    std::cout << "\n=====================";
    std::cout << "\n====print p1.m====\n";
    std::cout << p1.get_mass();
    std::cout << "\n====print p1.position ====\n";
    std::cout << p1.get_position();
    std::cout << "\n====print p1.velocity ====\n";
    std::cout << p1.get_velocity();
    std::cout << "\n";
}

void test_class_gridpt() {
    GridPoint g1 = GridPoint(10.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    std::cout << "\n===================";
    std::cout << "\n====TEST GRIDPT====";
    std::cout << "\n===================";
    std::cout << "\n====print g1.m====\n";
    std::cout << g1.get_mass();
    std::cout << "\n====print g1.position ====\n";
    std::cout << g1.get_position();
    std::cout << "\n====print g1.velocity ====\n";
    std::cout << g1.get_velocity();
    std::cout << "\n";
}

void test_1D_basis() {
    std::cout << "\n=======================";
    std::cout << "\n==== test 1D basis ====";
    std::cout << "\n=======================\n";
    for (double xi = -3.0; xi <= 3.0; xi += 0.15) {
        // TODO(yiminlin.tri): test partition of unity
        std::cout << "1Dbasis at xi=" << xi << " : "
                  << eval_1D_basis(xi) << std::endl;
    }
    std::cout << "\n============================";
    std::cout << "\n==== test 1D basis, POU ====";
    std::cout << "\n============================\n";
    // test partition of unity
    for (double xi = -1.0; xi <= 1.0; xi += 0.1) {
        std::cout << "1Dbasis at xi=" << xi << " : "
                  << eval_1D_basis(xi-2.0)+eval_1D_basis(xi+2.0)
                    +eval_1D_basis(xi-1.0)+eval_1D_basis(xi+1.0)
                    +eval_1D_basis(xi) << std::endl;
    }
}

void test_3D_basis() {
    // Grid of 5x5x5, index order (i, j, k), (x, y, z):
    // On [-2, 2]^3
    //         o - o - o - o - o
    //         |   |   |   |   |
    //     o - o - o - o - o - o
    //     |   |   |   |   |   |
    // o - o - o - o - o - o - o
    // |   |   |   |   |   |   |
    // o - o - o - o - o - o - o
    // |   |   |   |   |   |   |
    // o - o - o - o - o - o - o
    // |   |   |   |   |   |   |
    // o - o - o - o - o - o - o
    // |   |   |   |   |
    // o - o - o - o - o

    const drake::filesystem::path temp_dir = drake::temp_directory();
    const auto file = temp_dir / "test.bgeo";
    std::vector<drake::Vector3<double>> pos_arr, vel_arr;
    std::vector<double> val_arr;

    int i, j, k;
    double xi, yi, zi, u1, u2, u3, m;
    double sumval, h;
    drake::Vector3<double> tmpvec3{0.0, 0.0, 0.0};
    std::vector<GridPoint> gridpt_arr(125);
    m  = 0.0;
    u1 = 0.0;
    u2 = 0.0;
    u3 = 0.0;

    std::cout << "\n=======================";
    std::cout << "\n==== test 3D basis ====";
    std::cout << "\n=======================\n";

    // TODO(yiminlin.tri): this is only h = 1 case
    h = 1.0;
    for (int idx = 0; idx < 125; ++idx) {
        // TODO(yiminlin.tri):
        // refactor. Should be reused in other routines in the future
        i = idx % 5;
        j = (idx % 25) / 5;
        k = idx / 25;
        xi = -2.0 + i;
        yi = -2.0 + j;
        zi = -2.0 + k;
        gridpt_arr[idx] = GridPoint(m, xi, yi, zi, u1, u2, u3);
        // TODO(yiminlin.tri): Comment out, reuse later
        // std::cout << gridpt_arr[idx].get_position()(2) << std::endl;
        pos_arr.emplace_back(xi, yi, zi);
        vel_arr.emplace_back(0.0, 0.0, 0.0);
        // TODO(yiminlin.tri): hardcoded idx to output
        tmpvec3 = {xi, yi, zi};
        val_arr.emplace_back(gridpt_arr[62].eval_basis_3D(tmpvec3, h));
    }

    drake::multibody::mpm::internal::
        WriteParticlesToBgeo(file.string(), pos_arr, vel_arr, val_arr);

    std::cout << "\n===========================";
    std::cout << "\n==== test 3D basis POU ====";
    std::cout << "\n===========================\n";
    // For sampled points in [-1, 1]^3
    for (zi = -1.0; zi <= 1.0; zi += 0.2) {
    for (yi = -1.0; yi <= 1.0; yi += 0.2) {
    for (xi = -1.0; xi <= 1.0; xi += 0.2) {
        // Iterate through all grid points
        // (actually basis functions, and accumulate values)
        sumval = 0.0;
        tmpvec3 = {xi, yi, zi};
        for (int idx = 0; idx < 125; ++idx) {
            i = idx % 5;
            j = (idx % 25) / 5;
            k = idx / 25;
            sumval += gridpt_arr[idx].eval_basis_3D(tmpvec3, h);
        }

        // TODO(yiminlin.tri): hardcoded tolerance
        if (std::abs(sumval-1.0) > 1e-10) {
            std::cout << " ==== POU 3D ERR ====" << std::endl;
        }
    }
    }
    }
}

int main() {
    // TODO(yiminlin.tri): just for testing purposes
    //                     test using gtest
    // test_eigen_functionalities();
    // test_class_particle();
    // test_class_gridpt();
    // test_1D_basis();
    test_3D_basis();

    return 0;
}
