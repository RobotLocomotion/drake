#include "solvers/branch_and_bound.h"

MixedIntegerBranchAndBoundNode::MixedIntegerBranchAndBoundNode()
	: prog_(),
	left_child_{nullptr},
	right_child_{nullptr},
	parent_{nullptr},
	binary_var_index_{-1},
	binary_var_value_{-1} {}


MixedIntegerBranchAndBoundNode MixedIntegerBranchAndBoundNode::ConstructRootNode(const MathematicalProgram& prog) {

}


