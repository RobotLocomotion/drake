#include "drake/automotive/load_maliput_railcar_scenario_config_from_file.h"

#include <fcntl.h>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace drake {
namespace automotive {

void LoadMaliputRailcarScenarioConfigFromFile(
    const std::string& path, MaliputRailcarScenarioConfig* scenario_config) {
  int fid = open(path.data(), O_RDONLY);
  if (fid < 0) {
    throw std::runtime_error("LoadMaliputRailcarScenarioConfigFromFile:"
        " Cannot open file " + path + ".");
  }
  google::protobuf::io::FileInputStream istream(fid);
  if (!google::protobuf::TextFormat::Parse(&istream, scenario_config)) {
    throw std::runtime_error("LoadMaliputRailcarScenarioConfigFromFile:"
        " Error parsing " + path + ".");
  }
  istream.Close();
}

}  // namespace automotive
}  // namespace drake
