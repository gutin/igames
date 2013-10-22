#include "Extensions.hpp"
#include <cmath>
#include <limits>

namespace ig { namespace core {

double ImplementationUncertaintyEvaluator::evaluate(const UDC& udc_,
                                  const UDCNetwork& unet_,
                                  uvertex_t uv_,
                                  const Network& net_, 
                                  const State& state_,
                                  const ActionSharedPtr& candidate_,
                                  double totalRate_, size_t budget_,
                                  size_t fcode_, StateTemplateMap& stmap_) const
{
  if(state_._active.find(net_.end()) != state_._active.end() ||
     state_._dormant.find(net_.end()) != state_._dormant.end())
  {
    return 0;
  }
  if(candidate_->empty())
  {
    return _stdEval.evaluate(udc_, unet_, uv_, net_, state_, candidate_, totalRate_, budget_, fcode_, stmap_);
  }
  double value = 0;
  const size_t currentTau = tau(udc_, state_, budget_);
  for(size_t oc = 0; oc < (1L << candidate_->size()); ++oc)
  {
    double outcomeProb = 1;
                          
    size_t nextTau = currentTau;
    size_t count = 0;
    BOOST_FOREACH(vertex_t t, *candidate_)
    {
      if(oc & (1L << count))
      {
        outcomeProb *= TASK_ATTR(t, _probDelaySuccess);
        nextTau &= ~(1L << udc_._activity2UDCIndex[t]);
      }
      else
      {
        outcomeProb *= 1 - TASK_ATTR(t, _probDelaySuccess);
      }
      ++count;

    }
    // clear the budget bits in the tau value
    nextTau ^= (1L << (2*udc_.size())) * (budget_ - state_._res);
    nextTau |= (1L << (2*udc_.size())) * (budget_ - state_._res + count);
    value += udc_.value(nextTau) * outcomeProb;
  }
  return value;
}

double CrashingEvaluator::evaluate(const UDC& udc_,
                                  const UDCNetwork& unet_,
                                  uvertex_t uv_,
                                  const Network& net_, 
                                  const State& state_,
                                  const ActionSharedPtr& candidate_,
                                  double totalRate_, size_t budget_,
                                  size_t fcode_, StateTemplateMap& stmap_) const
{
  if(state_._active.find(net_.end()) != state_._active.end() ||
     state_._dormant.find(net_.end()) != state_._dormant.end())
  {
    return 0;
  }
  double minValue = std::numeric_limits<double>::max();
  
  double enumerator = 1;
  std::vector<double> nextStateVals;
  nextStateVals.reserve(state_._active.size());
  BOOST_FOREACH(vertex_t u, state_._active)
  {
    // optimised way of finding the value in the subsequent state
    double nextVal = 0;
    StateTemplate& nextST = nextTemplate(u, udc_, uv_, stmap_, net_, unet_, fcode_);
    const UDC& stUDC = unet_[nextST._udc]; 
    if(stUDC._taskSet.find(net_.end()) != stUDC._taskSet.end())
    {
      nextVal = 0;
    }
    else
    {
      size_t tav = (1L << (2* stUDC._tasks.size())) *
                 (budget_ - state_._res + candidate_->size());
      tav |= (1L << stUDC._tasks.size()) * nextST._activationCode;
      size_t tmp = (~0) - ((1L << stUDC._tasks.size()) - 1L) + nextST._activationCode;
      tav |= ~(tmp);

      BOOST_FOREACH(vertex_t t, *candidate_)
      {
        if(stUDC._taskSet.find(t) != stUDC._taskSet.end())
        {
          tav &= ~(1L << stUDC._activity2UDCIndex[t]);
        }
      }
      BOOST_FOREACH(vertex_t t, state_._interdicted)
      {
        if(stUDC._taskSet.find(t) != stUDC._taskSet.end())
        {
          tav &= ~(1L << stUDC._activity2UDCIndex[t]);
        }
      }
      if(stUDC._taskSet.find(u) != stUDC._taskSet.end())
      {
        tav &= ~(1L << stUDC._activity2UDCIndex[u]);
      }
      nextVal = stUDC.value(tav);
    }

    double rate = util::rate(u, net_, state_, candidate_);
    enumerator += nextVal * rate * Task::minInvestment();
    nextStateVals.push_back(nextVal);
  }

  size_t nextStateValIdx = 0;
  BOOST_FOREACH(vertex_t t, state_._active)
  {
    double rate = util::rate(t, net_, state_, candidate_);
    double delta = Task::maxInvestment() - Task::minInvestment();
    double val = (enumerator + rate * delta * nextStateVals[nextStateValIdx]) / (totalRate_ + rate * delta);
    minValue = std::min(minValue, val);
    ++nextStateValIdx;
  }
  return minValue;
}

}}
