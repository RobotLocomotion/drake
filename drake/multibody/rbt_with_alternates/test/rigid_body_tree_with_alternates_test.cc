#include "drake/multibody/rbt_with_alternates/rigid_body_tree_with_alternates.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/parser_urdf.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <utility>

using drake::AutoDiffXd;
using drake::RigidBodyTreeWithAlternates;

using std::complex;
using std::cout;
using std::endl;
using std::unique_ptr;
using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

int main() {
  // Fill in the "MultibodyTree" first.
  auto tree = make_unique<RigidBodyTree<double>>();

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());

#if 0
  tree->AddJoint(make_unique<PinJoint<double>>(1.1));
  tree->AddJoint(make_unique<SliderJoint<double>>(2.2));
  tree->AddJoint(make_unique<PinJoint<double>>(3.3));
  tree->AddJoint(make_unique<SliderJoint<double>>(4.4));
#endif

  // Create the fundamental MBSystem (that is, type double).
  RigidBodyTreeWithAlternates<double> tree_with_alternates(std::move(tree));

  // Create some alternate instantiations of MBSystem (kept within the
  // fundamental system).
  // Adds double alternate so that I can request it below.
  RigidBodyTreeWithAlternates<double>::AddAlternate(tree_with_alternates);
  RigidBodyTreeWithAlternates<AutoDiffXd>::AddAlternate(tree_with_alternates);

  cout << "num alternates=" << tree_with_alternates.get_num_alternates() << endl;

  //const auto& tree_double = tree_with_alternates.get_rigid_body_tree();
  const auto& tree_double =
      tree_with_alternates.get_alternate<double>().get_rigid_body_tree();
  const auto& tree_dynamics_autodiff =
      tree_with_alternates.get_alternate<AutoDiffXd>().get_rigid_body_tree();

  PRINT_VAR(tree_with_alternates.get_alternate<double>().type());
  PRINT_VAR(tree_with_alternates.get_alternate<AutoDiffXd>().type());

  PRINT_VAR(tree_double.get_num_bodies());
  PRINT_VAR(tree_dynamics_autodiff.get_num_bodies());

  PRINT_VAR(tree_double.get_num_positions());
  PRINT_VAR(tree_dynamics_autodiff.get_num_positions());

#if 0
  MBSystem<float>::AddAlternate(sys);



  // Stuff happens using the fundamental system, then at some point we decide
  // we want one of the alternate instantiations.

  const auto& csys = sys.get_alternate<complex<double>>();
  const auto& fsys = sys.get_alternate<float>();

  cout << "my type=" << sys.type() << endl;
  cout << "csys type=" << csys.type() << endl;
  cout << "fsys type=" << fsys.type() << endl;

  // Dig out the matching instantiations of the multibody tree.
  const auto& dtree = sys.get_tree();   // <double> (fundamental)
  const auto& ftree = fsys.get_tree();  // <float> (not useful)

  // Using the fundamental system, calculate derivative df analytically.
  Context<double> cd{0.5};  // set x=0.5 (set up context for fundamental)

  const auto& dpin0 = dtree.GetJoint<PinJoint>(0);
  double f = dpin0.PinFunc(cd);
  double df = dpin0.DPinFuncDx(cd);  // analytical derivative

  // Instead, use the same joint of the complex alternate to calculate the
  // derivative using a complex step derivative (equivalent to autodiff).
  const auto& ctree = csys.get_tree();
  const auto& cpin0 = ctree.GetJoint<PinJoint>(0);

  Context<complex<double>> cc(cd); // clone context for this alternate.
  cc.x += complex<double>(0, 1e-20);  // complex step derivative
  double cdf = cpin0.PinFunc(cc).imag() / 1e-20;

  printf("  f(x)=%.16g;\n df(x)=%.16g (analytical)\ncdf(x)=%.16g (autodiff)\n",
         f, df, cdf);
#endif

  //getchar();
}
