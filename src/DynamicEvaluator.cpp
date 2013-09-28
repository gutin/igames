#include "Utils.hpp"

namespace ig { namespace core {

template <class Evaluator>
template <class Policy>
double 
DynamicEvaluatorT<Evaluator>::evaluate(const Network& net_, 
                                       size_t budget_,
                                       const Policy& policy_) 
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
                                              const OrderedTaskSet& finished_) 
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
  typename Policy::const_iterator a_it = policy_.find(state);
  if(a_it == policy_.end())
  {
    std::cerr << "Warning failed to find action in state " << statePtr_->asString() << std::endl;
    return 0;
  }
//  std::cout << "Found the action in state " << statePtr_->asString() << std::endl;
  const ActionSharedPtr actionPtr = a_it->second;
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
                                              const OrderedTaskSet& finished_) 
{
  double secondTerm = 0, totalRate = 0;
  BOOST_FOREACH(vertex_t u, statePtr_->_active)
  {
    OrderedTaskSet newFinished = finished_;
    newFinished.insert(u);

    StateSharedPtr nextStatePtr = util::nextState(net_, statePtr_, actionPtr_, newFinished, u);
    double rate = victims_.find(u) != victims_.end() ? net_.graph()[u]._delta
                                                     : net_.graph()[u]._nu;
    totalRate += rate;
    secondTerm += rate * evaluateInState(net_, policy_, nextStatePtr, newFinished);
  }
  return (1 + secondTerm) / totalRate;
}

}}