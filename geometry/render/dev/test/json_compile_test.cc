#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main(int /* argc */, char* /* argv */[]) {
  auto j = json::parse(R"({"test": 11})");
  // If the code works and runs, we desire a return code of 0.
  return j["test"] != 11;
}
