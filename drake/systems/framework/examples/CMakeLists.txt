# This directory has no library targets. All the source code here is part
# drakeSystemFramework, which is declared in systems/framework.

drake_install_headers(
  controlled_spring_mass_system.h
  spring_mass_system.h)

add_subdirectory(test)
