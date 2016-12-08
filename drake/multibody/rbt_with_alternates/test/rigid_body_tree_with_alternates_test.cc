#include "drake/multibody/rbt_with_alternates/rigid_body_tree_with_aleternates.h"

int main() {
  // Fill in the "MultibodyTree" first.
  auto mb_tree = make_unique<RigidBodyTree<double>>();

#if 0
  mb_tree->AddJoint(make_unique<PinJoint<double>>(1.1));
  mb_tree->AddJoint(make_unique<SliderJoint<double>>(2.2));
  mb_tree->AddJoint(make_unique<PinJoint<double>>(3.3));
  mb_tree->AddJoint(make_unique<SliderJoint<double>>(4.4));

  // Create the fundamental MBSystem (that is, type double).
  MBSystem<double> sys(std::move(mb_tree));

  // Create some alternate instantiations of MBSystem (kept within the
  // fundamental system).
  MBSystem<complex<double>>::AddAlternate(sys);
  MBSystem<float>::AddAlternate(sys);

  cout << "num alternates=" << sys.get_num_alternates() << endl;

  // Stuff happens using the fundamental system, then at some point we decide
  // we want one of the alternate instantiations.

  const auto& csys = sys.get_alternate<complex<double>>();
  const auto& fsys = sys.get_alternate<float>();

  cout << "my type=" << sys.type() << endl;
  cout << "csys type=" << csys.type() << endl;
  cout << "fsys type=" << fsys.type() << endl;

  // Dig out the matching instantiations of the multibody tree.
  const auto& dtree = sys.get_mb_tree();   // <double> (fundamental)
  const auto& ftree = fsys.get_mb_tree();  // <float> (not useful)

  // Using the fundamental system, calculate derivative df analytically.
  Context<double> cd{0.5};  // set x=0.5 (set up context for fundamental)

  const auto& dpin0 = dtree.GetJoint<PinJoint>(0);
  double f = dpin0.PinFunc(cd);
  double df = dpin0.DPinFuncDx(cd);  // analytical derivative

  // Instead, use the same joint of the complex alternate to calculate the
  // derivative using a complex step derivative (equivalent to autodiff).
  const auto& ctree = csys.get_mb_tree();
  const auto& cpin0 = ctree.GetJoint<PinJoint>(0);

  Context<complex<double>> cc(cd); // clone context for this alternate.
  cc.x += complex<double>(0, 1e-20);  // complex step derivative
  double cdf = cpin0.PinFunc(cc).imag() / 1e-20;

  printf("  f(x)=%.16g;\n df(x)=%.16g (analytical)\ncdf(x)=%.16g (autodiff)\n",
         f, df, cdf);
#endif

  getchar();
}
