#include "drake/systems/primitives/linear_system.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/default_scalars.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/event_collection.h"

namespace drake {
namespace systems {

using std::make_unique;
using std::unique_ptr;

template <typename T>
LinearSystem<T>::LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              double time_period)
    : LinearSystem<T>(
          SystemTypeTag<systems::LinearSystem>{},
          A, B, C, D, time_period) {}

template <typename T>
template <typename U>
LinearSystem<T>::LinearSystem(const LinearSystem<U>& other)
    : LinearSystem<T>(
          other.A(), other.B(), other.C(), other.D(), other.time_period()) {}

template <typename T>
LinearSystem<T>::LinearSystem(SystemScalarConverter converter,
                              const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              double time_period)
    : AffineSystem<T>(
          std::move(converter),
          A, B, Eigen::VectorXd::Zero(A.rows()), C, D,
          Eigen::VectorXd::Zero(C.rows()), time_period) {}

template <typename T>
unique_ptr<LinearSystem<T>> LinearSystem<T>::MakeLinearSystem(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& output,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& state_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& input_vars,
    const double time_period) {
  // Need to extract, A, B, C, D such that,
  //
  //     dynamics = Ax + Bu
  //     output   = Cx + Du
  //
  // where x = state_vars and u = input_vars.
  const int num_states = state_vars.size();
  DRAKE_ASSERT(num_states == dynamics.size());
  const int num_inputs = input_vars.size();
  const int num_outputs = output.size();

  Eigen::MatrixXd AB(num_states, num_states + num_inputs);
  VectorX<symbolic::Variable> vars(num_states + num_inputs);
  vars << state_vars, input_vars;
  DecomposeLinearExpressions(dynamics, vars, &AB);
  const auto A = AB.leftCols(num_states);
  const auto B = AB.rightCols(num_inputs);

  Eigen::MatrixXd CD(num_outputs, num_states + num_inputs);
  DecomposeLinearExpressions(output, vars, &CD);
  const auto C = CD.leftCols(num_states);
  const auto D = CD.rightCols(num_inputs);

  return make_unique<LinearSystem<T>>(A, B, C, D, time_period);
}

namespace {

// If the system has discrete states, checks that the sole registered event for
// the system is periodic, and returns the time_period. A time_period of zero is
// returned if the system has continuous states.
double GetTimePeriodIfDiscreteUpdatesArePeriodic(
    const System<double>& system, const Context<double>& context) {
  if (!context.has_only_discrete_state()) return 0.;

  std::unique_ptr<CompositeEventCollection<double>> event_info =
      system.AllocateCompositeEventCollection();
  const double time_period =
      system.CalcNextUpdateTime(context, event_info.get()) - context.get_time();

  // Verify that the system has only one discrete, periodic update event.
  // TODO(jadecastro) Upon resolution of #6878, clean up this implementation and
  // weed out all illegal systems having a combination of event handlers.
  DRAKE_THROW_UNLESS(event_info->HasDiscreteUpdateEvents());
  const auto leaf_info =
      dynamic_cast<const LeafCompositeEventCollection<double>*>(
          event_info.get());
  DRAKE_DEMAND(leaf_info != nullptr);
  auto discrete_events = leaf_info->get_discrete_update_events().get_events();
  DRAKE_THROW_UNLESS(discrete_events.size() == 1);
  DRAKE_THROW_UNLESS(discrete_events.front()->get_trigger_type() ==
                     Event<double>::TriggerType::kPeriodic);
  return time_period;
}

/// XXX Intro
///
/// For each derivative in a problem, there are five indices -- its overall
/// index, its group index and its index within its group, and its chunk
/// index and its index within its chunk.
///
/// Say we have an problem with a first group of derivatives of size 4 and a
/// second group of derivatives of size 6 -- for a total of 10 derivatives, so:
///
/// - the OverallDerIndex spans [0, 9];
/// - the GroupIndex spans [0, 1];
/// - the GroupDersIndex for group 0 spans [0, 3];
/// - the GroupDersIndex for group 1 spans [0, 4].
///
/// With kChunkSize = 3, we will need ceil(10 / 3) = 4 chunks to cover it, so:
///
/// - the ChunkIndex spans [0, 3];
/// - the ChunkDerIndex spans [0, 2];
///
/// In tabular form, the indices match up as follows:
///
/// O'all | Group | Group | Chunk | Chunk
/// Der   |       | Der   |       | Der
/// Index | Index | Index | Index | Index
/// ----- | ----- | ----- | ----- | -----
/// 0     | 0     | 0     | 0     | 0
/// 1     | 0     | 1     | 0     | 1
/// 2     | 0     | 2     | 0     | 2
/// 3     | 0     | 3     | 1     | 0
/// 4     | 1     | 0     | 1     | 1
/// 5     | 1     | 1     | 1     | 2
/// 6     | 1     | 2     | 2     | 0
/// 7     | 1     | 3     | 2     | 1
/// 8     | 1     | 4     | 2     | 2
/// 9     | 1     | 5     | 3     | 0
/// -     | -     | -     | 3     | 1
/// -     | -     | -     | 3     | 2
///
/// In the above table, the choice for the ChunkIndex & ChunkDerIndex for a
/// given OverallDerIndex is simply counting up by one each row and then
/// wrapping.  Calling code should not depend on this, however, because
/// ChunkingAutodiff is allowed to partition the indices into chunks however it
/// likes.  For example, the two spares at the end of the table could have been
/// moved to the middle, so that group 0 was in chunks 0 & 1 (plus two spares)
/// and group 1 was fit precisely into chunks 2 & 3 (with no spares).
template <int kChunkSize>
class ChunkingAutodiff {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ChunkingAutodiff)

  /// The fixed-size AutoDiffScalar that we will use for all computation.
  using ADS = AutoDiffd<kChunkSize>;

  /// An overall index referring to all derivatives (across all groups).  The
  /// range is [0, total_num_derivatives()).
  using OverallDerIndex = TypeSafeIndex<class OverallDerIndexTag>;

  /// An index referring to groups of derivatives.  For example, ∂/∂x might be
  /// group 0, while ∂/∂u might be group 1.  The range is [0, ...).
  using GroupIndex = TypeSafeIndex<class GroupIndexTag>;

  /// An index referring to an offset with a specific group's derivatives.  The
  /// range is [0, size(g)) where g is a GroupIndex and size(g) is the argument
  /// to AddDerivativesGroup when g was declared.
  using GroupDerIndex = TypeSafeIndex<class GroupDerIndexTag>;

  /// An index referring to a fixed-size chunk of derivatives.  The range is
  /// [0, num_chunks) where num_chunks is defined as ⌈total_num_dervatives() /
  /// kChunkSize⌉.
  using ChunkIndex = TypeSafeIndex<class ChunkIndexTag>;

  /// An index referring to an offset within a fixed-sized chunk.  The range is
  /// [0, kChunkSize).
  using ChunkDerIndex = TypeSafeIndex<class ChunkDerIndexTag>;

  ChunkingAutodiff() {}
  ~ChunkingAutodiff() {}

  /// Add a group of derivatives (aka independent variables) to the problem.
  /// @param size non-negative count of derivatives in this group; a zero-size
  /// group will generate a valid unique GroupIndex but will not generate any
  /// gradients.
  GroupIndex AddDerivativesGroup(int size) {
    DRAKE_THROW_UNLESS(size >= 0);
    const GroupIndex result = num_groups();
    group_starts_.emplace_back(total_num_derivatives());
    group_sizes_.emplace_back(size);
    return result;
  }

  GroupIndex num_groups() const {
    return GroupIndex{static_cast<int>(group_starts_.size())};
  }

  ChunkIndex num_chunks() const {
    return ChunkIndex{
      (total_num_derivatives() + kChunkSize - 1) / kChunkSize};
  }

  /// Returns the total number of derivatives.  (That is, the sum of the
  /// argument over all calls to AddDerivativesGroup.)
  OverallDerIndex total_num_derivatives() const {
    OverallDerIndex result{0};
    if (!group_starts_.empty()) {
      // TODO(jwnimmer-tri) Would like to use TSI::operator+(int).
      result = group_starts_.back();
      result += int{group_sizes_.back()};
    }
    return result;
  }

  /// Resets the derivatives()s of all of `vec`'s elements to be either 0 or 1.
  /// Only those elements that are independent variables within the current
  /// chunk are set to 1, otherwise they are set to 0.  This method does *NOT*
  /// change group_value's elements' value()s at all; it *only* changes their
  /// derivatives()s.
  void InitializeDerivatives(
      ChunkIndex chunk, GroupIndex group, VectorBase<ADS>* vec) const {
    DRAKE_THROW_UNLESS(is_valid(chunk));
    DRAKE_THROW_UNLESS(is_valid(group));
    DRAKE_THROW_UNLESS(vec != nullptr);
    const GroupDerIndex group_size = group_sizes_[group];
    DRAKE_THROW_UNLESS(vec->size() == group_size);
    for (auto i = GroupDerIndex{0}; i < group_size; ++i) {
      (*vec)[i].derivatives().setZero();
    }
    for (auto item : get_common_range(chunk, group)) {
      GroupDerIndex i = item.group_der_index();
      (*vec)[i].derivatives()[item.chunk_der_index()] = 1.0;
    }
  }

  /// Copies some of the derivatives of `values` with respect to `group` into
  /// `gradients`.  This assumes that the computation that produced `values`
  /// was seeded with the given `chunk` via `InitializeDerivatives(chunk,
  /// ...)`.  Only the derivatives drawn from the given `chunk` and `group` are
  /// copied.  To obtain all derivatives of `values` with respect to `group`,
  /// this method must in invoked for all chunks.  Elements of `gradients` that
  /// do not map to the current `chunk` will remain unchanged.
  void ExtractGradients(
      ChunkIndex chunk, GroupIndex group, const VectorBase<ADS>& values,
      EigenPtr<Eigen::MatrixXd> gradients) const {
    DRAKE_THROW_UNLESS(is_valid(chunk));
    DRAKE_THROW_UNLESS(is_valid(group));
    DRAKE_THROW_UNLESS(gradients != nullptr);
    const int values_size = values.size();
    const GroupDerIndex group_size = group_sizes_[group];
    DRAKE_THROW_UNLESS_EQ(gradients->rows(), values_size);
    DRAKE_THROW_UNLESS_EQ(gradients->cols(), group_size);
    const auto& common_range = get_common_range(chunk, group);
    for (int i = 0; i < values_size; ++i) {
      const ADS& value = values[i];
      for (auto item : common_range) {
        const GroupDerIndex group_index = item.group_der_index();
        const ChunkDerIndex chunk_index = item.chunk_der_index();
        (*gradients)(i, group_index) = value.derivatives()(chunk_index);
      }
    }
  }

 private:
  static_assert(kChunkSize > 0, "The chunk size must be strictly positive");

  class DerRange {
   public:
    class Iter;

    class Der {
     public:
      GroupDerIndex group_der_index() const {
        // TODO(jwnimmer-tri) Would like to use TSI::operator+(int).
        return GroupDerIndex{parent_->group_offset + i_};
      }

      ChunkDerIndex chunk_der_index() const {
        // TODO(jwnimmer-tri) Would like to use TSI::operator+(int).
        return ChunkDerIndex{parent_->chunk_offset + i_};
      }

#if 0
      OverallDerIndex overall_der_index() const {
        return OverallDerIndex{
          (parent_->chunk * kChunkSize) + parent_->chunk_offset + i_};
      }
#endif

     private:
      friend class Iter;
      Der(const DerRange* parent, int i) : parent_{parent}, i_{i} {}

      const DerRange* parent_{};
      int i_{};
    };

    class Iter : public std::iterator<
      std::bidirectional_iterator_tag, const Der> {
     public:
      // (Purposefully omit unwanted operator++(int).)
      Iter& operator++() { ++i_; return *this; }
      bool operator==(Iter other) const { return (i_ == other.i_); }
      bool operator!=(Iter other) const { return !(*this == other); }
      Der operator*() const { return Der{parent_, i_}; }

     private:
      friend class DerRange;
      Iter(const DerRange* parent, int i) : parent_{parent}, i_{i} {}

      const DerRange* parent_{};
      int i_{};
    };

    Iter begin() const { return Iter{this, 0}; }
    Iter end() const { return Iter{this, size}; }

   private:
    friend class ChunkingAutodiff;

    ChunkIndex chunk;
    GroupIndex group;
    ChunkDerIndex chunk_offset;
    GroupDerIndex group_offset;
    int size{};
  };

  bool is_valid(ChunkIndex chunk) const {
    return (chunk * kChunkSize) <= total_num_derivatives();
  }

  bool is_valid(GroupIndex group) const {
    return group < num_groups();
  }

  // Assumes is_valid is all set.
  DerRange get_common_range(ChunkIndex chunk, GroupIndex group) const {
    DerRange result;
    result.chunk = chunk;
    result.group = group;
    const OverallDerIndex chunk_begin{chunk * kChunkSize};
    const OverallDerIndex chunk_end{(chunk + 1) * kChunkSize};
    const OverallDerIndex group_begin{group_starts_[group]};
    const OverallDerIndex group_end{group_begin + group_sizes_[group]};
    const OverallDerIndex common_begin = std::max(chunk_begin, group_begin);
    const OverallDerIndex common_end = std::min(chunk_end, group_end);
    if (common_end > common_begin) {
      result.size = common_end - common_begin;
      result.chunk_offset = common_begin - chunk_begin;
      result.group_offset = common_begin - group_begin;
      DRAKE_ASSERT(result.size >= 0);
      DRAKE_ASSERT(result.chunk_offset >= 0);
      DRAKE_ASSERT(result.group_offset >= 0);
    }
    return result;
  }

  std::vector<GroupDerIndex> group_sizes_;
  std::vector<OverallDerIndex> group_starts_;
};

template class ChunkingAutodiff<6>;

// A is extracted from gradients of xdot or x[n+1] wrt x
// B is extracted from gradients of xdot or x[n+1] wrt u.
// C is extracted from gradients of y wrt x
// D is extracted from gradients of y wrt u.
template <class FixedSizeAutoDiffScalar>
class Linearizer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Linearizer)

  Linearizer(const System<double>& system, const Context<double>& context)
      : // Determine the attributes of the System to be linearized.  NOLINT
        // The helpers will throw if the System's shape is unsupported.
        num_inputs_{determine_num_inputs(system)},
        num_outputs_{determine_num_outputs(system)},
        num_states_{determine_num_states(context)},
        has_only_continuous_state_{context.has_only_continuous_state()},
        has_only_discrete_state_{context.has_only_discrete_state()},
        time_period_{determine_time_period(system, context)},
         // Convert the System to AutoDiff form.
        autodiff_system_{Transmogrify(system)},
        autodiff_context_{autodiff_system_->CreateDefaultContext()},
        autodiff_u0_storage_{FixInput(num_inputs_, autodiff_context_.get())} {
    DRAKE_ASSERT_VOID(system.CheckValidContext(context));

    // Populate the operating point values.
    autodiff_context_->SetTimeStateAndParametersFrom(context);
    if (num_inputs() > 0) {
      GetMutableInput().SetFromVector(
          system.EvalEigenVectorInput(context, 0).template cast<ADS>());
    }
  }

  int num_inputs() const { return num_inputs_; }
  int num_outputs() const { return num_outputs_; }
  int num_states() const { return num_states_; }
  double time_period() const { return time_period_; }

  // Verify the equilibrium condition.
  void CheckForEquilibrium(double tolerance) const {
    const char* const kMessage =
        "The nominal operating point (x0,u0) is not an equilibrium point of "
        "the system.  Without additional information, a time-invariant "
        "linearization of this system is not well defined.";
    if (has_only_continuous_state_) {
      // Ensure that xdot0 = f(x0,u0) == 0.
      const VectorBase<ADS>& autodiff_xdot0 = EvalStateEquation();
      const VectorX<double>& xdot0 =
          math::autoDiffToValueMatrix(autodiff_xdot0.CopyToVector());
      if (!xdot0.isZero(tolerance)) {
        throw std::runtime_error(kMessage);
      }
    } else if (has_only_discrete_state_) {
      // Ensure that x1 = f(x0,u0) == x0.
      const VectorX<double>& x0 =
          math::autoDiffToValueMatrix(GetState().CopyToVector());
      const VectorBase<ADS>& autodiff_x1 = EvalStateEquation();
      const VectorX<double>& x1 =
          math::autoDiffToValueMatrix(autodiff_x1.CopyToVector());
      if (!(x1 - x0).isZero(tolerance)) {
        throw std::runtime_error(kMessage);
      }
    } else {
      DRAKE_DEMAND(num_states() == 0);
    }
  }

  void Linearize(EigenPtr<Eigen::MatrixXd> A, EigenPtr<Eigen::MatrixXd> B,
                 EigenPtr<Eigen::MatrixXd> C, EigenPtr<Eigen::MatrixXd> D) {
    // If any output arguments are zero-dimensional, handle that now.
    PrepareLinearizeResult(num_states(), num_states(), &A);
    PrepareLinearizeResult(num_states(), num_inputs(), &B);
    PrepareLinearizeResult(num_outputs(), num_states(), &C);
    PrepareLinearizeResult(num_outputs(), num_inputs(), &D);

    // If all output arguments are nullptr, we have nothing left to do.
    if (!(A || B || C || D)) { return; }

    // Figure out which and how many AutoDiff derivatives we want.
    Chunker chunker;
    const GroupIndex x_group =
        chunker.AddDerivativesGroup((A || C) ? num_states() : 0);
    const GroupIndex u_group =
        chunker.AddDerivativesGroup((B || D) ? num_inputs() : 0);

    // Read out the answer, one chunk at a time.
    for (auto chunk = ChunkIndex{0}; chunk < chunker.num_chunks(); ++chunk) {
      LinearizeChunk(chunker, chunk, x_group, u_group, A, B, C, D);
    }
  }

 private:
  using ADS = FixedSizeAutoDiffScalar;
  enum : int { kChunkSize = int{ADS::DerType::RowsAtCompileTime} };
  static_assert(kChunkSize > 0,
                "FixedSizeAutoDiffScalar must be of fixed non-zero size");
  static_assert(ADS::DerType::ColsAtCompileTime == 1,
                "FixedSizeAutoDiffScalar only accepts column vectors");
  using Chunker = ChunkingAutodiff<kChunkSize>;
  using ChunkIndex = typename Chunker::ChunkIndex;
  using GroupIndex = typename Chunker::GroupIndex;

  static int determine_num_inputs(const System<double>& system) {
    const int port_count = system.get_num_input_ports();
    if (port_count == 0) { return 0; }
    DRAKE_THROW_UNLESS(port_count == 1);
    return system.get_input_port(0).size();
  }

  static int determine_num_outputs(const System<double>& system) {
    const int port_count = system.get_num_output_ports();
    if (port_count == 0) { return 0; }
    DRAKE_THROW_UNLESS(port_count == 1);
    return system.get_output_port(0).size();
  }

  static int determine_num_states(const Context<double>& context) {
    DRAKE_THROW_UNLESS(
        context.is_stateless() ||
        context.has_only_continuous_state() ||
        (context.has_only_discrete_state() &&
         context.get_num_discrete_state_groups() == 1));
    return context.get_num_total_states();
  }

  static double determine_time_period(
      const System<double>& system, const Context<double>& context) {
    return GetTimePeriodIfDiscreteUpdatesArePeriodic(system, context);
  }

  static std::unique_ptr<System<ADS>> Transmogrify(
      const System<double>& system) {
    std::unique_ptr<System<ADS>> result =
        system.get_system_scalar_converter().
        Convert<ADS, double>(system);
    DRAKE_THROW_UNLESS(result != nullptr);
    return result;
  }

  static FreestandingInputPortValue* FixInput(
      int size, Context<ADS>* autodiff_context) {
    if (size == 0) {
      return nullptr;
    } else {
      return &autodiff_context->FixInputPort(0, VectorX<ADS>::Zero(size));
    }
  }

  void PrepareLinearizeResult(
      int rows, int cols, EigenPtr<Eigen::MatrixXd>* result) {
    DRAKE_DEMAND(result != nullptr);
    // If the user didn't pass this argument, we're already done.
    if (*result == nullptr) {
      return;
    }
    // If the problem would produce a zero-size result, we're already done.
    if ((rows == 0) || (cols == 0)) {
      *result = nullptr;
      return;
    }
    // Confirm the user passed in a right-size result, then it with NaNs.
    EigenPtr<Eigen::MatrixXd> target = *result;
    DRAKE_THROW_UNLESS_EQ(target->rows(), rows);
    DRAKE_THROW_UNLESS_EQ(target->cols(), cols);
    *target = Eigen::MatrixXd::Constant(
        rows, cols, Eigen::NumTraits<double>::quiet_NaN());
  }

  // Sugar to access the autodiff system's fixed input `u`.
  VectorBase<ADS>& GetMutableInput() {
    DRAKE_ASSERT(autodiff_u0_storage_ != nullptr);
    BasicVector<ADS>* const basic_vector =
        autodiff_u0_storage_->GetMutableVectorData<ADS>();
    DRAKE_ASSERT(basic_vector != nullptr);
    return *basic_vector;
  }

  // Sugar to access the autodiff system's state `x`.
  const VectorBase<ADS>& GetState() const {
    if (has_only_continuous_state_) {
      return autodiff_context_->get_continuous_state_vector();
    } else if (has_only_discrete_state_) {
      return *autodiff_context_->get_discrete_state(0);
    }
    DRAKE_ABORT();
  }

  // Sugar to access the autodiff system's state `x`.
  VectorBase<ADS>& GetMutableState() {
    if (has_only_continuous_state_) {
      return *autodiff_context_->get_mutable_continuous_state_vector();
    } else if (has_only_discrete_state_) {
      return *autodiff_context_->get_mutable_discrete_state(0);
    }
    DRAKE_ABORT();
  }

  // Compute the autodiff system's output `y`.
  const VectorBase<ADS>& EvalOutput() const {
    // Latch-initialize our storage on demand.
    if (!autodiff_y0_storage_) {
      autodiff_y0_storage_ =
          autodiff_system_->AllocateOutput(*autodiff_context_);
    }
    // Obtain the output y and its partial derivatives.
    autodiff_system_->CalcOutput(
        *autodiff_context_, autodiff_y0_storage_.get());
    // Sanity check for nullptrs on the way out.
    const BasicVector<ADS>* const result =
        autodiff_y0_storage_->get_vector_data(0);
    DRAKE_ASSERT(result != nullptr);
    return *result;
  }

  // Compute the autodiff system's state derivative or update;
  // this is `xdot` when continuous, or `x[n+1]` when discrete.
  const VectorBase<ADS>& EvalStateEquation() const {
    if (has_only_continuous_state_) {
      // Latch-initialize our storage on demand.
      if (!autodiff_xdot_storage_) {
        autodiff_xdot_storage_ = autodiff_system_->AllocateTimeDerivatives();
      }
      // Obtain state derivative xdot and its partial derivatives.
      autodiff_system_->CalcTimeDerivatives(
          *autodiff_context_, autodiff_xdot_storage_.get());
      return autodiff_xdot_storage_->get_vector();
    } else if (has_only_discrete_state_) {
      // Latch-initialize our storage on demand.
      if (!autodiff_x1_storage_) {
        autodiff_x1_storage_ = autodiff_system_->AllocateDiscreteVariables();
      }
      // Obtain state update xd[n+1] and its partial derivatives.
      autodiff_system_->CalcDiscreteVariableUpdates(
          *autodiff_context_, autodiff_x1_storage_.get());
      const BasicVector<ADS>* const result =
          autodiff_x1_storage_->get_vector(0);
      // Sanity check for nullptrs on the way out.
      DRAKE_ASSERT(result != nullptr);
      return *result;
    }
    DRAKE_ABORT();
  }

  // The chunker must always be the same object.
  void LinearizeChunk(
      const Chunker& chunker,
      const ChunkIndex chunk,
      const GroupIndex x_group,
      const GroupIndex u_group,
      const EigenPtr<Eigen::MatrixXd> A,
      const EigenPtr<Eigen::MatrixXd> B,
      const EigenPtr<Eigen::MatrixXd> C,
      const EigenPtr<Eigen::MatrixXd> D) {
    // Re-initialize the partials for this chunk's input and state.
    VectorBase<ADS>& autodiff_x0 = GetMutableState();
    chunker.InitializeDerivatives(chunk, x_group, &autodiff_x0);
    VectorBase<ADS>& autodiff_u0 = GetMutableInput();
    chunker.InitializeDerivatives(chunk, u_group, &autodiff_u0);

    // Read out the chunk-wise contribution to A,B,C,D.
    if (A || B) {
      const VectorBase<ADS>& autodiff_xdot = EvalStateEquation();
      if (A) { chunker.ExtractGradients(chunk, x_group, autodiff_xdot, A); }
      if (B) { chunker.ExtractGradients(chunk, u_group, autodiff_xdot, B); }
    }
    if (C || D) {
      const VectorBase<ADS>& autodiff_y0 = EvalOutput();
      if (C) { chunker.ExtractGradients(chunk, x_group, autodiff_y0, C); }
      if (D) { chunker.ExtractGradients(chunk, u_group, autodiff_y0, D); }
    }
  }

  // Properties of the System being linearized.
  const int num_inputs_;
  const int num_outputs_;
  const int num_states_;
  const bool has_only_continuous_state_;
  const bool has_only_discrete_state_;
  const double time_period_;

  // The AutoDiff-based flavor of the system to be linearized.
  const std::unique_ptr<const System<ADS>> autodiff_system_;
  const std::unique_ptr<Context<ADS>> autodiff_context_;
  FreestandingInputPortValue* const autodiff_u0_storage_;

  // Storage that is allocated on demand and then re-used.
  mutable std::unique_ptr<ContinuousState<ADS>> autodiff_xdot_storage_;
  mutable std::unique_ptr<DiscreteValues<ADS>> autodiff_x1_storage_;
  mutable std::unique_ptr<SystemOutput<ADS>> autodiff_y0_storage_;
};

}  // namespace

std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    double equilibrium_check_tolerance) {
  // Transmogrify the system and set its operating point.
  Linearizer<AutoDiff6d> linearizer{system, context};
  linearizer.CheckForEquilibrium(equilibrium_check_tolerance);

  // Extract the derivatives.
  Eigen::MatrixXd A(linearizer.num_states(), linearizer.num_states());
  Eigen::MatrixXd B(linearizer.num_states(), linearizer.num_inputs());
  Eigen::MatrixXd C(linearizer.num_outputs(), linearizer.num_states());
  Eigen::MatrixXd D(linearizer.num_outputs(), linearizer.num_inputs());
  linearizer.Linearize(&A, &B, &C, &D);

  return std::make_unique<LinearSystem<double>>(
      A, B, C, D, linearizer.time_period());
}

/// Returns the controllability matrix:  R = [B, AB, ..., A^{n-1}B].
Eigen::MatrixXd ControllabilityMatrix(const LinearSystem<double>& sys) {
  DRAKE_DEMAND(sys.time_period() == 0.0);
  // TODO(russt): handle the discrete time case

  const int num_states = sys.B().rows(), num_inputs = sys.B().cols();
  Eigen::MatrixXd R(num_states, num_states * num_inputs);
  R.leftCols(num_inputs) = sys.B();
  for (int i = 1; i < num_states; i++) {
    R.middleCols(num_inputs * i, num_inputs) =
        sys.A() * R.middleCols(num_inputs * (i - 1), num_inputs);
  }
  return R;
}

/// Returns true iff the controllability matrix is full row rank.
bool IsControllable(const LinearSystem<double>& sys, double threshold) {
  const auto R = ControllabilityMatrix(sys);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> lu_decomp(R);
  lu_decomp.setThreshold(threshold);
  return lu_decomp.rank() == sys.A().rows();
}

/// Returns the observability matrix: O = [ C; CA; ...; CA^{n-1} ].
Eigen::MatrixXd ObservabilityMatrix(const LinearSystem<double>& sys) {
  DRAKE_DEMAND(sys.time_period() == 0.0);
  // TODO(russt): handle the discrete time case

  const int num_states = sys.C().cols(), num_outputs = sys.C().rows();
  Eigen::MatrixXd O(num_states * num_outputs, num_states);
  O.topRows(num_outputs) = sys.C();
  for (int i = 1; i < num_states; i++) {
    O.middleRows(num_outputs * i, num_outputs) =
        O.middleRows(num_outputs * (i - 1), num_outputs) * sys.A();
  }
  return O;
}

/// Returns true iff the observability matrix is full column rank.
bool IsObservable(const LinearSystem<double>& sys, double threshold) {
  const auto O = ObservabilityMatrix(sys);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> lu_decomp(O);
  lu_decomp.setThreshold(threshold);
  return lu_decomp.rank() == sys.A().rows();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LinearSystem)
