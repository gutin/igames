#include "Extensions.hpp"
#include <cmath>

namespace ig { namespace core {

double ImplementationUncertaintyEvaluator::evaluate(const UDC& udc_,
                                  const UDCNetwork& unet_,
                                  uvertex_t uv_,
                                  const Network& net_, 
                                  const State& state_,
                                  const ActionSharedPtr& candidate_,
                                  double totalRate_, size_t budget_,
                                  size_t fcode_, StateTemplateMap& stmap_)
{
  if(util::isTerminalState(net_, state_))
  {
    return 0;
  }
  if(candidate_->empty())
  {
    return StandardEvaluator::evaluate(udc_, unet_, uv_, net_, state_, candidate_, totalRate_, budget_, fcode_, stmap_);
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

}}
