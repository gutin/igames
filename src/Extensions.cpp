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
  double totalProb = 0;
  double value = 0;
  for(size_t oc = 0; oc < (1L << candidate_->size()); ++oc)
  {
    double outcomeProb = 1;
    State next = state_;
    size_t count = 0;
    BOOST_FOREACH(vertex_t t, *candidate_)
    {
      if(oc & (1L << count))
      {
        outcomeProb *= TASK_ATTR(t, _probDelaySuccess);
        next._interdicted.insert(t);
      }
      else
      {
        outcomeProb *= 1 - TASK_ATTR(t, _probDelaySuccess);
      }
      ++count;

    }
    next._res -= candidate_->size();
    totalProb += outcomeProb;
    size_t tav = tau(udc_, next, budget_);
    value += udc_.value(tav) * outcomeProb;
  }
  assert(std::abs(totalProb - 1) < 1e-6);
  return value;
}

}}
