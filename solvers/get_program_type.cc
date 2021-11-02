#include "drake/solvers/get_program_type.h"

#include <initializer_list>
#include <vector>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/program_attribute.h"

namespace drake {
namespace solvers {
namespace {
// The requirements of a type of optimization program based on its attributes.
// The requirements are that the program attributes
// 1. is a subset of @p acceptable_attributes.
// 2. must include all of @p must_include_attributes.
// 3. must include at least one of @p must_include_one_of.
// If must_include_one_of is empty, then we ignore this condition.
struct Requirements {
  ProgramAttributes acceptable;
  ProgramAttributes must_include;
  ProgramAttributes must_include_one_of;
};
// Check if @p program_attributes satisfies @p requirements.
bool SatisfiesProgramType(const Requirements& requirements,
                          const ProgramAttributes& program_attributes) {
  // Check if program_attributes is a subset of acceptable_attributes
  for (const auto attribute : program_attributes) {
    if (requirements.acceptable.count(attribute) == 0) {
      return false;
    }
  }
  // Check if program_attributes include must_include_attributes
  for (const auto& must_include_attr : requirements.must_include) {
    if (program_attributes.count(must_include_attr) == 0) {
      return false;
    }
  }
  bool include_one_of = requirements.must_include_one_of.empty() ? true : false;
  for (const auto& include : requirements.must_include_one_of) {
    if (program_attributes.count(include) > 0) {
      include_one_of = true;
      break;
    }
  }
  if (!include_one_of) {
    return false;
  }
  return true;
}

const Requirements& GetRequirementsLP() {
  static const drake::never_destroyed<Requirements> requirements{[]() {
    ProgramAttributes attributes{std::initializer_list<ProgramAttribute>{
        ProgramAttribute::kLinearCost, ProgramAttribute::kLinearConstraint,
        ProgramAttribute::kLinearEqualityConstraint}};
    return Requirements{.acceptable = attributes,
                        .must_include = {},
                        .must_include_one_of = attributes};
  }()};
  return requirements.access();
}

const Requirements& GetRequirementsQP() {
  static const drake::never_destroyed<Requirements> requirements{[]() {
    const Requirements& lp_requirements = GetRequirementsLP();
    ProgramAttributes acceptable = lp_requirements.acceptable;
    acceptable.emplace(ProgramAttribute::kQuadraticCost);
    return Requirements{.acceptable = acceptable,
                        .must_include = {ProgramAttribute::kQuadraticCost},
                        .must_include_one_of = {}};
  }()};
  return requirements.access();
}

const Requirements& GetRequirementsSOCP() {
  static const drake::never_destroyed<Requirements> requirements{[]() {
    const Requirements& lp_requirements = GetRequirementsLP();
    ProgramAttributes acceptable = lp_requirements.acceptable;
    acceptable.emplace(ProgramAttribute::kLorentzConeConstraint);
    acceptable.emplace(ProgramAttribute::kRotatedLorentzConeConstraint);
    return Requirements{.acceptable = acceptable,
                        .must_include = {},
                        .must_include_one_of = {
                            ProgramAttribute::kLorentzConeConstraint,
                            ProgramAttribute::kRotatedLorentzConeConstraint}};
  }()};
  return requirements.access();
}

const Requirements& GetRequirementsSDP() {
  static const drake::never_destroyed<Requirements> requirements{[]() {
    const Requirements& socp_requirements = GetRequirementsSOCP();
    ProgramAttributes acceptable = socp_requirements.acceptable;
    acceptable.emplace(ProgramAttribute::kPositiveSemidefiniteConstraint);
    return Requirements{
        .acceptable = acceptable,
        .must_include = {ProgramAttribute::kPositiveSemidefiniteConstraint},
        .must_include_one_of = {}};
  }()};
  return requirements.access();
}

const Requirements& GetRequirementsGP() {
  static const drake::never_destroyed<Requirements> requirements{Requirements{
      .acceptable = {ProgramAttribute::kLinearCost,
                     ProgramAttribute::kExponentialConeConstraint},
      .must_include = {ProgramAttribute::kExponentialConeConstraint},
      .must_include_one_of = {}}};
  return requirements.access();
}

const Requirements& GetRequirementsCGP() {
  static const drake::never_destroyed<Requirements> requirements{[]() {
    const Requirements& sdp_requirements = GetRequirementsSDP();
    ProgramAttributes acceptable = sdp_requirements.acceptable;
    acceptable.emplace(ProgramAttribute::kExponentialConeConstraint);
    ProgramAttributes must_include_one_of = sdp_requirements.acceptable;
    must_include_one_of.erase(ProgramAttribute::kLinearCost);
    return Requirements{
        .acceptable = acceptable,
        .must_include = {ProgramAttribute::kExponentialConeConstraint},
        .must_include_one_of = must_include_one_of};
  }()};
  return requirements.access();
}

const Requirements& GetRequirementsQuadraticCostConicProgram() {
  static const drake::never_destroyed<Requirements> requirements{[]() {
    const Requirements& cgp_requirements = GetRequirementsCGP();
    ProgramAttributes acceptable = cgp_requirements.acceptable;
    acceptable.emplace(ProgramAttribute::kQuadraticCost);
    return Requirements{.acceptable = acceptable,
                        .must_include = {ProgramAttribute::kQuadraticCost},
                        .must_include_one_of = {
                            // All nonlinear convex conic constraints.
                            ProgramAttribute::kLorentzConeConstraint,
                            ProgramAttribute::kRotatedLorentzConeConstraint,
                            ProgramAttribute::kPositiveSemidefiniteConstraint,
                            ProgramAttribute::kExponentialConeConstraint}};
  }()};
  return requirements.access();
}

const Requirements& GetRequirementsLCP() {
  static const drake::never_destroyed<Requirements> requirements{Requirements{
      .acceptable = {ProgramAttribute::kLinearComplementarityConstraint},
      .must_include = {ProgramAttribute::kLinearComplementarityConstraint},
      .must_include_one_of = {}}};
  return requirements.access();
}

// Returns the requirements of a mixed-integer optimization program.
// Append kBinaryVariable to acceptable and must_include.
// Retain must_include_one_of.
Requirements MixedIntegerRequirements(
    const Requirements& continuous_requirements) {
  Requirements mip_requirements = continuous_requirements;
  mip_requirements.acceptable.emplace(ProgramAttribute::kBinaryVariable);
  mip_requirements.must_include.emplace(ProgramAttribute::kBinaryVariable);
  return mip_requirements;
}

const Requirements& GetRequirementsMILP() {
  static const drake::never_destroyed<Requirements> requirements{
      []() { return MixedIntegerRequirements(GetRequirementsLP()); }()};
  return requirements.access();
}

const Requirements& GetRequirementsMIQP() {
  static const drake::never_destroyed<Requirements> requirements{
      []() { return MixedIntegerRequirements(GetRequirementsQP()); }()};
  return requirements.access();
}

const Requirements& GetRequirementsMISOCP() {
  static const drake::never_destroyed<Requirements> requirements{
      []() { return MixedIntegerRequirements(GetRequirementsSOCP()); }()};
  return requirements.access();
}

const Requirements& GetRequirementsMISDP() {
  static const drake::never_destroyed<Requirements> requirements{
      []() { return MixedIntegerRequirements(GetRequirementsSDP()); }()};
  return requirements.access();
}

bool AllQuadraticCostsConvex(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs) {
  return std::all_of(quadratic_costs.begin(), quadratic_costs.end(),
                     [](const Binding<QuadraticCost>& quadratic_cost) {
                       return quadratic_cost.evaluator()->is_convex();
                     });
}

bool IsNLP(const MathematicalProgram& prog) {
  // A program is a nonlinear program (NLP) if it satisfies all the following
  // conditions:
  // 1. It has no binary variables.
  // 2. It is not an LCP (A program with only linear complementarity
  // constraints).
  // 3. It has at least one of : generic costs, generic constraints, non-convex
  // quadratic cost, linear complementarity constraints.
  const bool has_generic_cost =
      prog.required_capabilities().count(ProgramAttribute::kGenericCost) > 0;
  const bool has_nonconvex_quadratic_cost =
      !AllQuadraticCostsConvex(prog.quadratic_costs());
  const bool has_generic_constraint =
      prog.required_capabilities().count(ProgramAttribute::kGenericConstraint) >
      0;
  const bool no_binary_variable = prog.required_capabilities().count(
                                      ProgramAttribute::kBinaryVariable) == 0;
  const bool has_linear_complementarity_constraint =
      prog.required_capabilities().count(
          ProgramAttribute::kLinearComplementarityConstraint) > 0;
  const bool is_LCP =
      SatisfiesProgramType(GetRequirementsLCP(), prog.required_capabilities());
  return no_binary_variable && !is_LCP &&
         (has_generic_cost || has_nonconvex_quadratic_cost ||
          has_generic_constraint || has_linear_complementarity_constraint);
}
}  // namespace

ProgramType GetProgramType(const MathematicalProgram& prog) {
  if (SatisfiesProgramType(GetRequirementsLP(), prog.required_capabilities())) {
    return ProgramType::kLP;
  } else if (SatisfiesProgramType(GetRequirementsQP(),
                                  prog.required_capabilities()) &&
             AllQuadraticCostsConvex(prog.quadratic_costs())) {
    return ProgramType::kQP;
  } else if (SatisfiesProgramType(GetRequirementsSOCP(),
                                  prog.required_capabilities())) {
    return ProgramType::kSOCP;
  } else if (SatisfiesProgramType(GetRequirementsSDP(),
                                  prog.required_capabilities())) {
    return ProgramType::kSDP;
  } else if (SatisfiesProgramType(GetRequirementsGP(),
                                  prog.required_capabilities())) {
    // TODO(hongkai.dai): support more general type of geometric programming,
    // with constraints on posynomials.
    return ProgramType::kGP;
  } else if (SatisfiesProgramType(GetRequirementsCGP(),
                                  prog.required_capabilities())) {
    return ProgramType::kCGP;
  } else if (SatisfiesProgramType(GetRequirementsMILP(),
                                  prog.required_capabilities())) {
    return ProgramType::kMILP;
  } else if (SatisfiesProgramType(GetRequirementsMIQP(),
                                  prog.required_capabilities()) &&
             AllQuadraticCostsConvex(prog.quadratic_costs())) {
    return ProgramType::kMIQP;
  } else if (SatisfiesProgramType(GetRequirementsMISOCP(),
                                  prog.required_capabilities())) {
    return ProgramType::kMISOCP;
  } else if (SatisfiesProgramType(GetRequirementsMISDP(),
                                  prog.required_capabilities())) {
    return ProgramType::kMISDP;
  } else if (SatisfiesProgramType(GetRequirementsQuadraticCostConicProgram(),
                                  prog.required_capabilities()) &&
             AllQuadraticCostsConvex(prog.quadratic_costs())) {
    return ProgramType::kQuadraticCostConicConstraint;
  } else if (SatisfiesProgramType(GetRequirementsLCP(),
                                  prog.required_capabilities())) {
    return ProgramType::kLCP;
  } else if (IsNLP(prog)) {
    return ProgramType::kNLP;
  } else {
    return ProgramType::kUnknown;
  }
  DRAKE_UNREACHABLE();
}
}  // namespace solvers
}  // namespace drake
