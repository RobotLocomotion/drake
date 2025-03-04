#include "drake/systems/framework/bus_value.h"

#include "absl/container/flat_hash_map.h"

#include "drake/common/drake_export.h"

namespace drake {
namespace systems {

class DRAKE_NO_EXPORT BusValue::Impl {
 public:
  Impl();

  // We are copyable (and thus technically "moveable") but nothing else.
  Impl(const Impl& other);
  Impl& operator=(const Impl& other) = delete;

  // This methods are forwarded to us from the BusValue outer class (or its
  // nested Iterator helper class).
  BusValue::Iterator IterBegin() const;
  void IterAdvance(BusValue::Iterator* iterator) const;
  BusValue::Iterator::value_type IterGet(size_t index) const;
  const AbstractValue* Find(std::string_view name) const;
  void Clear();
  void Set(std::string_view name, const AbstractValue& value);

 private:
  // The storage for a single singal value. When valid is false, we must treat
  // the slot like it's empty. (We don't delete unused slots becuase they are
  // likely to become populated again.)
  struct Slot {
    std::string name;
    copyable_unique_ptr<AbstractValue> datum;
    bool valid{false};
  };

  // Fills index_ with a map of all names in data_.
  // @pre index_.empty()
  void RecomputeIndex();

  // The storage for a BusValue.
  std::vector<Slot> data_;

  // O(1) index from a Slot::name to that slot's index in data_.
  // This indexes all slots in data_, even the invalid ones.
  absl::flat_hash_map<std::string_view, size_t> index_;
};

BusValue::Impl::Impl() {
  // Avoid re-indexing for small sizes.
  data_.reserve(8);
  index_.reserve(8);
}

BusValue::Impl::Impl(const Impl& other) : Impl() {
  // We'll use the copy constructor as our signal to garbage collect unused
  // names, by only copying valid slots.
  for (const auto& slot : other.data_) {
    if (slot.valid) {
      data_.push_back(slot);
    }
  }
  RecomputeIndex();
}

BusValue::Iterator BusValue::Impl::IterBegin() const {
  for (size_t i = 0; i < data_.size(); ++i) {
    if (data_[i].valid) {
      Iterator result;
      result.index_ = i;
      result.impl_ = this;
      return result;
    }
  }
  return BusValue::Iterator{};
}

void BusValue::Impl::IterAdvance(BusValue::Iterator* iterator) const {
  for (size_t i = iterator->index_ + 1; i < data_.size(); ++i) {
    if (data_[i].valid) {
      iterator->index_ = i;
      return;
    }
  }
  *iterator = BusValue::Iterator{};
}

BusValue::Iterator::value_type BusValue::Impl::IterGet(size_t index) const {
  const Slot& slot = data_.at(index);
  DRAKE_THROW_UNLESS(slot.valid);
  DRAKE_ASSERT(slot.datum != nullptr);
  return {slot.name, *slot.datum};
}

const AbstractValue* BusValue::Impl::Find(std::string_view name) const {
  auto iter = index_.find(name);
  if (iter == index_.end()) {
    return nullptr;
  }
  const size_t position = iter->second;
  const Slot& slot = data_.at(position);
  if (!slot.valid) {
    return nullptr;
  }
  const AbstractValue* result = slot.datum.get();
  DRAKE_ASSERT(result != nullptr);
  return result;
}

void BusValue::Impl::Clear() {
  for (auto& slot : data_) {
    slot.valid = false;
  }
}

void BusValue::Impl::Set(std::string_view name, const AbstractValue& value) {
  auto iter = index_.find(name);
  if (iter == index_.end()) {
    Slot slot{.name = std::string{name}};
    slot.datum = value.Clone();
    slot.valid = true;
    const size_t position = data_.size();
    if (position < data_.capacity()) {
      data_.push_back(std::move(slot));
      index_.emplace(data_.back().name, position);
    } else {
      data_.push_back(std::move(slot));
      index_.clear();
      RecomputeIndex();
    }
  } else {
    const size_t position = iter->second;
    Slot& slot = data_.at(position);
    slot.datum->SetFrom(value);
    slot.valid = true;
  }
}

void BusValue::Impl::RecomputeIndex() {
  DRAKE_DEMAND(index_.empty());
  for (size_t i = 0; i < data_.size(); ++i) {
    index_.emplace(data_[i].name, i);
  }
}

// This is cheap enough to be inline in the header, but due to the forward
// declaration of our Impl class, it can't actually be defined there.
BusValue::BusValue() = default;

BusValue::BusValue(const BusValue&) = default;

BusValue& BusValue::operator=(const BusValue&) = default;

BusValue::BusValue(BusValue&&) = default;

BusValue& BusValue::operator=(BusValue&&) = default;

BusValue::~BusValue() = default;

BusValue::Iterator BusValue::begin() const {
  if (impl_ == nullptr) {
    return end();
  }
  return impl_->IterBegin();
}

const AbstractValue* BusValue::Find(std::string_view name) const {
  if (impl_ == nullptr) {
    return nullptr;
  }
  return impl_->Find(name);
}

void BusValue::Clear() {
  if (impl_ == nullptr) {
    return;
  }
  return impl_->Clear();
}

void BusValue::Set(std::string_view name, const AbstractValue& value) {
  if (impl_ == nullptr) {
    impl_ = std::make_unique<Impl>();
  }
  return impl_->Set(name, value);
}

BusValue::Iterator::value_type BusValue::Iterator::operator*() const {
  DRAKE_THROW_UNLESS(impl_ != nullptr);
  return static_cast<const BusValue::Impl*>(impl_)->IterGet(index_);
}

BusValue::Iterator& BusValue::Iterator::operator++() {
  DRAKE_THROW_UNLESS(impl_ != nullptr);
  static_cast<const BusValue::Impl*>(impl_)->IterAdvance(this);
  return *this;
}

BusValue::Iterator BusValue::Iterator::operator++(int) {
  Iterator result = *this;
  ++(*this);
  return result;
}

}  // namespace systems
}  // namespace drake
