#ifndef DYNAMICEVALUATOR_H
#define DYNAMICEVALUATOR_H

#include "Network.hpp"
#include "State.hpp"

#include <boost/unordered_map.hpp>

namespace ig { namespace core {

template <class Evaluator>
class DynamicEvaluatorT 
{
public:
  template <class Policy>
  double evaluate(const Network& net_, size_t budget_, const Policy& policy_) const;
protected:
  typedef boost::unordered_map<StateSharedPtr, double, StateSharedPtrHash> ValueCache;

  template <class Policy>
  double evaluateInState(const Network& net_, 
                         const Policy& policy_,
                         const StateSharedPtr& statePtr_,
                         const OrderedTaskSet& finished_) const;

  template <class Policy>
  double evaluateInStateImplBase(const Network& net_,
                                 const StateSharedPtr& statePtr_,
                                 const ActionSharedPtr& actionPtr_,
                                 const OrderedTaskSet& vicitms_,
                                 const Policy& policy_,
                                 const OrderedTaskSet& finished_) const;
  mutable ValueCache _vcache;
};

class StandardDynamicEvaluator : public DynamicEvaluatorT<StandardDynamicEvaluator>
{
public:
  template <class Policy>
  double evaluateInStateImpl(const Network& net_,
                             const StateSharedPtr& statePtr_,
                             const ActionSharedPtr& actionPtr_,
                             const OrderedTaskSet& vicitms_,
                             const Policy& policy_,
                             const OrderedTaskSet& finished_) const;
};

class ImplUncertaintyEvaluator : public DynamicEvaluatorT<ImplUncertaintyEvaluator>
{
public:
  template <class Policy>
  double evaluateInStateImpl(const Network& net_,
                             const StateSharedPtr& statePtr_,
                             const ActionSharedPtr& actionPtr_,
                             const OrderedTaskSet& vicitms_,
                             const Policy& policy_,
                             const OrderedTaskSet& finished_) const;
};

}}

#include "DynamicEvaluator.cpp"

#endif
