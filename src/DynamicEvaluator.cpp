#include "Utils.hpp"

namespace ig { namespace core {

template <class Evaluator>
template <class Policy>
double 
DynamicEvaluatorT<Evaluator>::evaluate(const Network& net_, 
                                       size_t budget_,
                                       const Policy& policy_) const
{
  StateSharedPtr sptr(new State);
  util::startingState(net_, budget_, *sptr);
  double ret = evaluateInState(net_, policy_, sptr, OrderedTaskSet());
  _vcache.clear();
  return ret;
}

template <class Evaluator>
template <class Policy>
double 
DynamicEvaluatorT<Evaluator>::evaluateInState(const Network& net_, 
                                              const Policy& policy_,
                                              const StateSharedPtr& statePtr_,
                                              const OrderedTaskSet& finished_) const
{
  if(util::isTerminalState(net_, *statePtr_))
  {
    return 0;
  }
  ValueCache::iterator it;
  if((it = _vcache.find(statePtr_)) != _vcache.end())
  {
    return it->second;
  }
  const State& state = *statePtr_;
  ActionSharedPtr actionPtr = policy_.at(state);
  OrderedTaskSet victims;
  std::set_union(actionPtr->begin(), actionPtr->end(),
                 statePtr_->_interdicted.begin(), statePtr_->_interdicted.end(),
                 std::inserter(victims, victims.begin()));
  return _vcache[statePtr_] = 
    (static_cast<const Evaluator*>(this))->evaluateInStateImpl(net_, statePtr_,
                                                               actionPtr, victims,
                                                               policy_,
                                                               finished_);
}
template <class Policy>
double 
StandardDynamicEvaluator::evaluateInStateImpl(const Network& net_,
                                              const StateSharedPtr& statePtr_,
                                              const ActionSharedPtr& actionPtr_,
                                              const OrderedTaskSet& victims_,
                                              const Policy& policy_,
                                              const OrderedTaskSet& finished_) const 
{
  return evaluateInStateImplBase(net_, statePtr_, actionPtr_, victims_, policy_, finished_); 
}

template <class Evaluator>
template <class Policy>
double 
DynamicEvaluatorT<Evaluator>::evaluateInStateImplBase(const Network& net_,
                                                      const StateSharedPtr& statePtr_,
                                                      const ActionSharedPtr& actionPtr_,
                                                      const OrderedTaskSet& victims_,
                                                      const Policy& policy_,
                                                      const OrderedTaskSet& finished_) const 
{
  double secondTerm = 0, totalRate = 0;
  BOOST_FOREACH(vertex_t u, statePtr_->_active)
  {
    OrderedTaskSet newFinished = finished_;
    newFinished.insert(u);

    StateSharedPtr nextStatePtr = util::nextState(net_, *statePtr_, actionPtr_, newFinished, u);
    double rate = victims_.find(u) != victims_.end() ? net_.graph()[u]._delta
                                                     : net_.graph()[u]._nu;
    totalRate += rate;
    secondTerm += rate * evaluateInState(net_, policy_, nextStatePtr, newFinished);
  }
  return (1 + secondTerm) / totalRate;
}

template <class Policy>
double 
ImplUncertaintyEvaluator::evaluateInStateImpl(const Network& net_,
                                              const StateSharedPtr& statePtr_,
                                              const ActionSharedPtr& actionPtr_,
                                              const OrderedTaskSet& victims_,
                                              const Policy& policy_,
                                              const OrderedTaskSet& finished_) const 
{
  if(actionPtr_ && !actionPtr_->empty())
  {
    double overallValue = 0;
    for(size_t outcome = 0; outcome < (1L << actionPtr_->size()); ++outcome)
    {
      double outcomeProb = 1;
      StateSharedPtr nextStatePtr(new State(*statePtr_));
      size_t count = 0;
      BOOST_FOREACH(vertex_t t, *actionPtr_)
      {
        if((1L << count) & outcome)
        {
          nextStatePtr->_interdicted.insert(t);
          outcomeProb *= TASK_ATTR(t, _probDelaySuccess);
        }
        else
        {
          outcomeProb *= 1 - TASK_ATTR(t, _probDelaySuccess);
        }
        ++count;
      }
      nextStatePtr->_res -= actionPtr_->size();
      assert(nextStatePtr->_res >= 0);
      OrderedTaskSet newFinished = finished_;
      overallValue += outcomeProb * evaluateInState(net_, policy_, nextStatePtr, newFinished);
    }
    return overallValue;
  }
  return  evaluateInStateImplBase(net_, statePtr_, actionPtr_, victims_, policy_, finished_);
}

template <class PT, class SE>
template <class DSP>
double FastEvaluator<PT,SE>::visitState(const Network net_, 
                                        size_t budget_,
                                        const UDCNetwork& unet_,
                                        const UDC& udc_,
                                        vertex_i uPtr_,
                                        const State& s,
                                        const OrderedTaskSet& active, 
                                        int fcode,
                                        StateTemplateMap& stmap,
                                        DSP& storagePolicy_)
{
  ActionSharedPtr actionPtr = _policy.at(s);

  double totalRate = 0;
  BOOST_FOREACH(vertex_t v, s._active)
  {
    totalRate += util::rate(v, net_, s, actionPtr);
  }
  return _stateEval.evaluate(udc_, unet_, *uPtr_, net_, s,actionPtr,totalRate,budget_,fcode,stmap); 
}

}}
