#include "Extensions.hpp"
#include <cmath>
#include <limits>

namespace ig { namespace core {

double ImplementationUncertaintyEvaluator::evaluate(const UDC& udc_,
                                  const UDCNetwork& unet_,
                                  uvertex_t uv_,
                                  const Network& net_, 
                                  const State& state_,
                                  const Action& candidate_,
                                  double totalRate_, size_t budget_,
                                  size_t fcode_, StateTemplateMap& stmap_) const
{
  if(state_._active.find(net_.end()) != state_._active.end() ||
     state_._dormant.find(net_.end()) != state_._dormant.end())
  {
    return 0;
  }
  if(candidate_.empty())
  {
    return _stdEval.evaluate(udc_, unet_, uv_, net_, state_, candidate_, totalRate_, budget_, fcode_, stmap_);
  }
  double value = 0;
  const size_t currentTau = tau(udc_, state_, budget_);
  for(size_t oc = 0; oc < (1L << candidate_.size()); ++oc)
  {
    double outcomeProb = 1;
                          
    size_t nextTau = currentTau;
    size_t count = 0;
    BOOST_FOREACH(vertex_t t, candidate_.asTaskSet())
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
                                  const Action& candidate_,
                                  double totalRate_, size_t budget_,
                                  size_t fcode_, StateTemplateMap& stmap_) const
{
  if(state_._active.find(net_.end()) != state_._active.end() ||
     state_._dormant.find(net_.end()) != state_._dormant.end())
  {
    return 0;
  }
  // The value of a state-decision pair in the crashing game is worked out as follows:
  // We know that the optimal renewable resource allocation for the proliferator is a corner-point of a polyhedron
  // So we iterate through each task in the current active set:
  //  1. Pretend that the proliferator puts as much as he can into it (taking into account min. investment into each task constraint)
  //  2. Lets also assume that the exact amount of the renewable resource is the the max number of tasks that can run in parallel
  //  3. So based on this information we know what all the possibilities are for the scaling of each task's duration
  // We minimize over all valus of these possiblities and essentially solve the proliferator's minimization subproblem
  
  // The following is a 'helper quantity' - we work out what would be the total cost of renewable rsource if we invested the minimum possible
  // in each task
  double residualCost = 0;
  BOOST_FOREACH(vertex_t t, state_._active)
  {
    residualCost += Task::MIN_INVESTMENT * TASK_ATTR(t, _crashingCost);
  }

  // Begin the pointwise minimization
  double minValue = std::numeric_limits<double>::max();
  BOOST_FOREACH(vertex_t t, state_._active)
  {
    // Enumerator and denominator in the final expression
    double enu = 1, denom = 0;

    // If the proliferator wuld put as much as possible into task 't' it would cost
    // ('total renewable resource' - ('the above helper quantity' - 'min investment cost into t')) * 'unit cost of crashing t'
    double crashingAmount = (unet_._maxParallel - residualCost + Task::MIN_INVESTMENT * TASK_ATTR(t, _crashingCost)) / TASK_ATTR(t, _crashingCost);  

    double totalRenewableResourceExpended = 0;
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
                   (budget_ - state_._res + candidate_.size());
        tav |= (1L << stUDC._tasks.size()) * nextST._activationCode;
        size_t tmp = (~0) - ((1L << stUDC._tasks.size()) - 1L) + nextST._activationCode;
        tav |= ~(tmp);

        BOOST_FOREACH(vertex_t t, candidate_.asTaskSet())
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

      // Found the value in the subsequnt state and xi is the amount of RR put in
      double xi = t == u ? crashingAmount : Task::MIN_INVESTMENT;
      totalRenewableResourceExpended += xi * TASK_ATTR(u, _crashingCost);
//      assert(xi >= Task::MIN_INVESTMENT - 1e-04); 

      double rate = util::rate(u, net_, state_, candidate_);

      // Build up the final expression
      enu += nextVal * rate * xi;
      denom += rate * xi; 
    }
    assert(std::abs(totalRenewableResourceExpended - unet_._maxParallel) < 1e-3);
    assert(denom > 0);
    // THe final expression....
    double val = enu / denom;
    // .. minimize over it.
    minValue = std::min(minValue, val);
  }
  return minValue;
}

}}
