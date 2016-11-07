#include "drake/multibody/parser_model_instance_id_table.h"

#include <stdexcept>

namespace drake {
namespace parsers {

void AddModelInstancesToTable(const ModelInstanceIdTable& source_table,
    ModelInstanceIdTable* dest_table) {
  for (auto const &model_entry : source_table) {
    const std::string& model_name = model_entry.first;
    if (dest_table->find(model_name) != dest_table->end()) {
      throw std::runtime_error("AddModelInstancesToTable: Collision occured "
          "with model name\"" + model_name + "\".");
    }
    (*dest_table)[model_name] = model_entry.second;
  }
}

}  // namespace parsers
}  // namespace drake
