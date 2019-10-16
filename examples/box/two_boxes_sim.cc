#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"
#include "drake/examples/box/two_boxes_plant.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/common/eigen_types.h"

#include <fstream>
//#include <experimental/filesystem>

namespace drake {
namespace examples {
namespace box {
namespace {
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(penalty_k, 300.0,
              "k stiffness constant for contact penalty force (N/m)");

DEFINE_double(penalty_d, 10.0,
              "d damping constant for contact penalty force (s/m)");

DEFINE_double(box1_init_x, -0.1,
              "initial position for box 1 (m)");
DEFINE_double(box1_init_v, 0.2,
              "initial velocity for box 1 (m/s)");

DEFINE_double(box2_init_x, 0.1,
              "initial position for box 2 (m)");
DEFINE_double(box2_init_v, 0.,
              "initial velocity for box 2 (m/s)");
DEFINE_double(box_m, 0.33,
              "box mass (kg)");

DEFINE_double(box_d, 0.2,
              "box damping (s^-1)");

DEFINE_double(box_l, 0.12,
              "box length (m)");


DEFINE_double(box1_target_x, 0.045,
              "box1 target position (m)");

DEFINE_double(box1_target_v, 0.0,
              "box1 target velocity (m/s)");

DEFINE_double(box2_target_x, 0.284,
              "box2 target position (m)");

DEFINE_double(box2_target_v, 0.0,
              "box2 target velocity (m/s)");

void StoreTwoBoxesEigenCSV(const std::string& filename, const VectorX<double>& times, const MatrixX<double>& data)
{
  /* csv format from  https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format */
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                  Eigen::DontAlignCols, ", ", "\n");
  std::ofstream file(filename);
  file << "t, box1x, box1v, box2x, box2v, box2u" << std::endl;
  /* horizontally concatenate times and data */
  MatrixX<double> OutMatrix(times.rows(), times.cols() + data.rows());
  OutMatrix << times, data.transpose(); /* can also do this with blocks */
  file << OutMatrix.format(CSVFormat);
  file.close();
}

void TestTwoBoxesEigenCSV()
{
  std::string filename = "testcsv.csv";
  /*boost::filesystem::path p{filename};
  if(boost::filesystem::create_directory(p.parent_path()))
  {
    std::cout << "directory created!" << std::endl;
  }*/

  Eigen::MatrixXd datamatrix(6,5);
  datamatrix << 11, 12, 13, 14, 15,
  21, 22, 23, 24, 25,
  31, 32, 33, 34, 35,
  41, 42, 43, 44, 45,
  51, 52, 53, 54, 55,
  61, 62, 63, 64, 65;
  Eigen::VectorXd timematrix(6);
  timematrix << 1,2,3,4,5,6;
  StoreTwoBoxesEigenCSV(filename, timematrix.block(0,0,5,1), datamatrix.block(0,0,5,5));
}
int DoMain() {
  TestTwoBoxesEigenCSV();
  systems::DiagramBuilder<double> builder;

  auto source1 = builder.AddSystem<systems::Sine>(4 * M_PI * M_PI * 1.0 * 0. /* amplitude */, 
                                                  2 * M_PI /* omega */, M_PI / 2.0 /* phase */, 1 /* vector size */);
  source1->set_name("source1");
  /* target box location */
  auto target1 = builder.AddSystem<BoxPlant>(0.0 /* mass */, FLAGS_box_l /* length */, 0.0 /* damping */);
  auto target2 = builder.AddSystem<BoxPlant>(0.0 /* mass */, FLAGS_box_l /* length */, 0.0 /* damping */);
  auto two_boxes = builder.AddSystem<TwoBoxesPlant>(FLAGS_box_m,
      FLAGS_box_d, FLAGS_box_l, FLAGS_penalty_k, FLAGS_penalty_d);
  auto logger = LogOutput(two_boxes->get_log_output(), &builder);
      // ADD AND CONNECT LOGGER TO SYSTEM
  builder.Connect(source1->get_output_port(0), two_boxes->get_input_port(0));
  builder.Connect(source1->get_output_port(0), target1->get_input_port());
  builder.Connect(source1->get_output_port(0), target2->get_input_port());
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  // add targets last
  AddGeometryToBuilder(&builder, *two_boxes, scene_graph);
  BoxGeometry::AddToBuilder(&builder, *target1, scene_graph,"3");
  BoxGeometry::AddToBuilder(&builder, *target2, scene_graph,"4");
  ConnectDrakeVisualizer(&builder, *scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& context = diagram->GetMutableSubsystemContext(*two_boxes,
                                          &simulator.get_mutable_context());
  systems::Context<double>& t1context = diagram->GetMutableSubsystemContext(*target1,
                                          &simulator.get_mutable_context());
  systems::Context<double>& t2context = diagram->GetMutableSubsystemContext(*target2,
                                          &simulator.get_mutable_context());
  
  drake::VectorX<double> initState1(2);
  initState1 << FLAGS_box1_init_x, FLAGS_box1_init_v ;
  drake::VectorX<double> initState2(2);
  initState2 << FLAGS_box2_init_x, FLAGS_box2_init_v;
  two_boxes->SetBox1State(&context, initState1);
  two_boxes->SetBox2State(&context, initState2);
  drake::VectorX<double> targetState1(2);
  targetState1 << FLAGS_box1_target_x, FLAGS_box1_target_v;
  drake::VectorX<double> targetState2(2);
  targetState2 << FLAGS_box2_target_x, FLAGS_box2_target_v;
  target1->set_initial_state(&t1context, targetState1);
  target2->set_initial_state(&t2context, targetState2);

  const double initial_energy = two_boxes->CalcBoxesTotalEnergy(context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(10);
  // print out log contents 
  StoreTwoBoxesEigenCSV("simout.csv", logger->sample_times(), logger->data());
  const double final_energy = two_boxes->CalcBoxesTotalEnergy(context);

  // Adds a numerical sanity test on total energy.
  DRAKE_DEMAND(initial_energy + 0.5 * M_PI * M_PI + 0.5 >=  final_energy);
  return 0;
}

}  // namespace
}  // namespace box
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::box::DoMain();
}
