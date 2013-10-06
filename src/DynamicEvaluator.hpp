#ifndef DYNAMICEVALUATOR_H
#define DYNAMICEVALUATOR_H

#include "Network.hpp"
#include "State.hpp"
#include "DynamicAlgorithm.hpp"

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

template <class PolicyType, class StateEvaluator>
class FastEvaluation
{
public:
  FastEvaluation(const PolicyType& policy_) :
    _policy(policy_) {}

  double evaluate(const UDC& udc, const UDCNetwork& unet_, uvertex_t uv_,
                  const Network& net_, 
                  const State& s, const ActionSharedPtr& candidate,
                  double totalRate, size_t budget_,
                  size_t fcode, StateTemplateMap& stmap);  
private:
  const PolicyType& _policy;

  StateEvaluator _stateEval;
};

template <class PolicyType, class StateEvaluator>
class FastEvaluator : public FastTraverserT<FastEvaluator<PolicyType, StateEvaluator> >
{
public:
  FastEvaluator(const PolicyType& policy_) : _policy(policy_) {}

  double evaluate(const Network& net_, size_t budget_)
  {
    NoDecisionStorage dsp;
    return this->execute(net_, budget_, dsp); 
  }

  template <class DSP>
  double visitState(const Network net_, 
                                        size_t budget_,
                                        const UDCNetwork& unet_,
                                        const UDC& udc_,
                                        vertex_i uPtr_,
                                        const State& s,
                                        const OrderedTaskSet& active, 
                                        int fcode,
                                        StateTemplateMap& stmap,
                                        DSP& storagePolicy_);
private:
  const PolicyType& _policy;

  StateEvaluator _stateEval;
};

}}

#include "DynamicEvaluator.cpp"

#endif
