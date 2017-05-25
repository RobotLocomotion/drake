#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcm/translator.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/translator_internal.h"

namespace drake {
namespace systems {
namespace lcm {

template <typename DataType, typename MsgType>
class ToLcmMessageTranslator : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ToLcmMessageTranslator)

  using TranslatorType = typename drake::lcm::TranslatorBase<DataType, MsgType>;

  // For VectorBase and it's derived.
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type>
  ToLcmMessageTranslator(const DataType1& default_input,
                         std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    DeclareVectorInputPort(default_input);

    Value<MsgType> msg;
    translator_->InitializeMessage(&(msg.template GetMutableValue<MsgType>()));
    DeclareAbstractOutputPort(msg);
  }

  // For every thing else.
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<!std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type,
            typename = void>
  ToLcmMessageTranslator(const DataType1& default_input,
                         std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    DeclareAbstractInputPort(Value<DataType>(default_input));

    Value<MsgType> msg;
    translator_->InitializeMessage(&(msg.template GetMutableValue<MsgType>()));
    DeclareAbstractOutputPort(msg);
  }

  const TranslatorType& get_translator() const { return *translator_; };

 private:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    const DataType& data = translator_internal::DataTypeTraits<
        DataType, std::is_base_of<VectorBase<double>,
                                  DataType>::value>::get_data(*this, context);

    MsgType& msg = output->GetMutableData(0)->GetMutableValue<MsgType>();

    translator_->Encode(context.get_time(), data, &msg);
  }

  std::unique_ptr<TranslatorType> translator_;
};

template <typename DataType, typename MsgType>
class FromLcmMessageTranslator : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FromLcmMessageTranslator)

  using TranslatorType = typename drake::lcm::TranslatorBase<DataType, MsgType>;

  template <typename DataType1 = DataType,
            typename = typename std::enable_if<std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type>
  FromLcmMessageTranslator(const DataType1& default_output,
                           std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    Value<MsgType> msg;
    translator_->InitializeMessage(&(msg.template GetMutableValue<MsgType>()));

    DeclareAbstractInputPort(msg);
    DeclareVectorOutputPort(default_output);
  }

  // For every thing else.
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<!std::is_base_of<
                systems::VectorBase<double>, DataType1>::value>::type,
            typename = void>
  FromLcmMessageTranslator(const DataType1& default_output,
                           std::unique_ptr<TranslatorType> translator)
      : translator_(std::move(translator)) {
    Value<MsgType> msg;
    translator_->InitializeMessage(&(msg.template GetMutableValue<MsgType>()));

    DeclareAbstractInputPort(msg);
    DeclareAbstractOutputPort(Value<DataType>(default_output));
  }

 private:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    const MsgType& msg =
        this->EvalAbstractInput(context, 0)->template GetValue<MsgType>();
    DataType& data = translator_internal::DataTypeTraits<
        DataType, std::is_base_of<VectorBase<double>,
                                  DataType>::value>::get_mutable_data(output);
    double throw_away_time;
    translator_->Decode(msg, &throw_away_time, &data);
  }

  std::unique_ptr<TranslatorType> translator_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
