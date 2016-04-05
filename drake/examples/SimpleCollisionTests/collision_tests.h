#ifndef COLLISION_TESTS_H_
#define COLLISION_TESTS_H_

#include <memory>
#include "drake/systems/plants/shapes/HeightMapTerrain.h"

namespace Drake{

class SinusoidalTerrain
    : public DrakeShapes::HeightMapTerrain {
 public:
  SinusoidalTerrain(const std::string& name, const Eigen::Vector2i& ncells,
                    const Eigen::Vector2d& size, double height = 1.0)
      : HeightMapTerrain(name, ncells, size), m_height(height) {
    for (int i = 0; i < nnodes(0); ++i) {
      double x = i * delta_ell(0);
      for (int j = 0; j < nnodes(1); ++j) {
        //double y = j * delta_ell(1);
        double z = m_height * sin(x * 3.1416 / size(0));
        cellValue(i, j) = z;
      }
    }
    this->computeMinMaxHeights();
  }
  SinusoidalTerrain(const SinusoidalTerrain& other) : HeightMapTerrain(other) {}
  SinusoidalTerrain* clone() const { return new SinusoidalTerrain(*this); }

 protected:
  double m_height;
};

}//namespace Drake

//forward declaration
namespace Drake{
  class RigidBodySystem;
}

class collisionTests{
public:
  collisionTests();
  collisionTests(const char* fname,bool flat_terrain,double angle, double x_pos,double t_final,double delt, bool with_visualizer=true);
  void run();
  double get_solution();
  collisionTests& set_time_step(double dt);
private:
  //case options
  bool flat_terrain;   //!< true for flat terrain test. False for height map test.
  bool with_visualizer;//!< true: uses bot visualizer to display solution. false: only runs simulate without cascading to a bot visualizer
  double angle;        //!< angle for flat terrain test.
  double x_pos;        //!< x position for height map terrain test.
  std::string urdf_file;
  double delt;         //!< the simulation time step
  double t_final;
  std::shared_ptr<Drake::RigidBodySystem> rigid_body_sys;
  Eigen::VectorXd x;   //<! state vector
};

#endif //COLLISION_TESTS_H_
