#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcm/translator_base.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * An encoding system that converts data of DataType to a Lcm message of
 * MsgType. This system has exactly one input port and one output port. The
 * output port is always of abstract value, which contains the encoded Lcm
 * message. If DataType is derived from systems::VectorBase<double>, the
 * input port will be vector valued. The input port will be abstract valued
 * otherwise. The input and output ports' model values are specified by the
 * translator instance passed to the constructor.
 */
template <typename DataType, typename MsgType>
class
    DRAKE_DEPRECATED(
        "The LcmAndVectorBaseTranslator and its related code are deprecated, "
        "and will be removed on 2019-05-01.")
    LcmEncoderSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmEncoderSystem)

  using TranslatorType = typename drake::lcm::TranslatorBase<DataType, MsgType>;

  /**
   * Constructor when DataType is derived from systems::VectorBase<double>.
   * Declares a vector valued input port and an abstract output port. Sets the
   * model values for the input and output ports to @p translator's
   * corresponding default values.
   *
   * @param translator Translator, whose ownership is transferred to this
   * instance.
   */
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type>
  explicit LcmEncoderSystem(std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    DeclareVectorInputPort(translator_->get_default_data());
    DeclareAbstractOutputPort(translator_->get_default_msg(),
        &LcmEncoderSystem::EncodeMsg);
  }

  /**
   * Constructor when DataType is not derived from systems::VectorBase<double>.
   * Declares an abstract valued input port and an abstract output port. Sets
   * the model values for the input and output ports to @p translator's
   * corresponding default values.
   *
   * @param translator Translator, whose ownership is transferred to this.
   */
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<!std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type,
            typename = void>
  explicit LcmEncoderSystem(std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    DeclareAbstractInputPort(Value<DataType1>(translator_->get_default_data()));
    DeclareAbstractOutputPort(translator_->get_default_msg(),
        &LcmEncoderSystem::EncodeMsg);
  }

  /**
   * Returns a const reference to the translator.
   */
  const TranslatorType& get_translator() const { return *translator_; }

 private:
  void EncodeMsg(const Context<double>& context, MsgType* msg) const {
    const auto& data = this->get_input_port(0).template Eval<DataType>(context);
    translator_->Encode(data, msg);
    translator_->EncodeTime(context.get_time(), msg);
  }

  std::unique_ptr<TranslatorType> translator_;
};

/**
 * A decoding system that converts a Lcm message of MsgTypedata to data of
 * DataType. This system has exactly one input port and one output port. The
 * input port is always of abstract value, which contains the encoded Lcm
 * message. If DataType is derived from systems::VectorBase<double>, the
 * output port will be vector valued. The output port will be abstract valued
 * otherwise. The input and output ports' model values are specified by the
 * translator instance passed to the constructor.
 */
template <typename DataType, typename MsgType>
class
    DRAKE_DEPRECATED(
        "The LcmAndVectorBaseTranslator and its related code are deprecated, "
        "and will be removed on 2019-05-01.")
    LcmDecoderSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDecoderSystem)

  using TranslatorType = typename drake::lcm::TranslatorBase<DataType, MsgType>;

  /**
   * Constructor when DataType is derived from systems::VectorBase<double>.
   * Declares an abstract input port and a vector valued output port. Sets the
   * model values for the input and output ports to @p translator's
   * corresponding default values.
   *
   * @param translator Translator, whose ownership is transferred to this.
   */
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type>
  explicit LcmDecoderSystem(std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    DeclareAbstractInputPort(Value<MsgType>(translator_->get_default_msg()));
    DeclareVectorOutputPort(
        translator_->get_default_data(),
        &LcmDecoderSystem::DecodeMsg);
  }

  /**
   * Constructor when DataType is not derived from systems::VectorBase<double>.
   * Declares an abstract input port and an abstract valued output port. Sets
   * the model values for the input and output ports to @p translator's
   * corresponding default values.
   *
   * @param translator Translator, whose ownership is transferred to this.
   */
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<!std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type,
            typename = void>
  explicit LcmDecoderSystem(std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    DeclareAbstractInputPort(Value<MsgType>(translator_->get_default_msg()));
    DeclareAbstractOutputPort(
        translator_->get_default_data(),
        &LcmDecoderSystem::DecodeMsg);
  }

  /**
   * Returns a const reference to the translator.
   */
  const TranslatorType& get_translator() const { return *translator_; }

 private:
  void DecodeMsg(const Context<double>& context, DataType* vector) const {
    const auto& msg = this->get_input_port(0).template Eval<MsgType>(context);
    translator_->Decode(msg, vector);
  }

  std::unique_ptr<TranslatorType> translator_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
