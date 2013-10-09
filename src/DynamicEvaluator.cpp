#include "Utils.hpp"
#include "DynamicAlgorithm.hpp"

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

template<class PT,class SE>
double FastEvaluator<PT,SE>::evaluate(const Network& net_, size_t budget_) const
{
  UDCNetwork unet(net_);
  size_t numUDCs = unet.size();
  size_t udcCount = 0, totalStates = 0;
  boost::progress_timer ptimer;
  
  UDCPtrs uPtrs;
  unet.sortedUDCs(uPtrs);

  for(size_t i = 0; i < numUDCs - 1; ++i)
  {
    uvertex_i uPtr = uPtrs[i];
    std::cout << "UDC[" << *uPtr << "] [" << (++udcCount) << "/" << numUDCs << "]" << std::endl;
    size_t states = solveUDC(uPtr, unet, net_, budget_);
    std::cout << "Solved UDC. Total of " << states << std::endl;
    totalStates += states;

    u_oute_i oe, oe_end;
    boost::tie(oe, oe_end) = boost::out_edges(*uPtr, unet._ug);
    for(; oe != oe_end; ++oe)
    {
      uvertex_t outUDC = boost::target(*oe, unet._ug);
      size_t& dependCount = unet[outUDC]._dependCount;
      if(--dependCount == 0)
      {
        std::cout << "\t --- Cleaning up UDC " << outUDC << std::endl;
        unet[outUDC].cleanup();
      }
    }
  }
  std::cout << "Solving final UDC - the start one " << std::endl; 
  uvertex_i startUdc = uPtrs[numUDCs-1];
  size_t states = solveUDC(startUdc, unet, net_, budget_);
  std::cout << "Last one had " << states << " states" << std::endl;
  totalStates += states;
  std::cout << "In total there were " << totalStates << " states" << std::endl;
  State startingState;
  util::startingState(net_, budget_, startingState);
  return unet[*startUdc].value(tau(unet._ug[*startUdc], startingState, budget_));
}

template<class PT,class SE>
size_t FastEvaluator<PT,SE>::solveUDC(vertex_i uPtr_, 
                                 UDCNetwork& unet_,
                                 const Network& net_,
                                 size_t budget_) const
{
  StateTemplateMap stmap;
  size_t stateCount(0);
  UDC& udc = unet_[*uPtr_];
  size_t maxFCode = (1L << udc.size()) - 2;
  std::cout << "Starting with a maximum finish code of " << maxFCode 
            << " for UDC [ ";
  std::copy(udc._tasks.begin(), udc._tasks.end(),
            std::ostream_iterator<vertex_t>(std::cout, " "));
  std::cout << "]" << std::endl;
  
  size_t N = udc.size();
  for(size_t y = 0; y <= budget_; ++y)
  {
    for(size_t fcode = maxFCode; (fcode + 1) >= 1; --fcode)
    {
      State s(y);
      size_t dormants = ((1L << N) -1) & fcode;

      for(size_t i = 0; i < N; ++i)
      {
        if(dormants & (1L << i))
        {
          s._dormant.insert(udc._tasks[i]);
        }
      }

      OrderedTaskSet eligible;
      BOOST_FOREACH(vertex_t t, s._dormant)
      {
        oute_i ei, ei_end;
        boost::tie(ei, ei_end) = boost::out_edges(t, net_.graph());
        for(;ei != ei_end;++ei)
        {
          vertex_t trg = boost::target(*ei, net_.graph());
          eligible.insert(trg);
        }
      }

      bool validFCode = true;
      BOOST_FOREACH(vertex_t t, eligible)
      {
        OrderedTaskSet unfinishedPreds;
        ine_i ei, ei_end;
        boost::tie(ei, ei_end) = boost::in_edges(t, net_.graph());
        for(;ei != ei_end;++ei)
        {
          vertex_t s = boost::source(*ei, net_.graph());
          if(udc._finished.find(s) == udc._finished.end())
          {
            unfinishedPreds.insert(s);
          }
        }

        bool dormantsContainsAllUPs = true;
        BOOST_FOREACH(vertex_t up, unfinishedPreds)
        {
          if(s._dormant.find(up) == s._dormant.end())
          {
            dormantsContainsAllUPs = false;
            break;
          }
        }
        if(dormantsContainsAllUPs)
        {
          validFCode = false;
          break;
        }
      }
      if(!validFCode) continue;
        
      std::set_difference(udc._taskSet.begin(), udc._taskSet.end()
                          ,s._dormant.begin(), s._dormant.end(),
                          std::inserter(s._active, s._active.begin()));

      OrderedTaskSet active;
      size_t maxActiveCode = (1L << s._active.size()) - 1;
      for(int activeCombo = maxActiveCode; (activeCombo + 1) >= 1; --activeCombo)
      {
        size_t realActiveCode = 0;
        OrderedTaskSet active;
        size_t lastActive = 0;
        for(size_t i = 0 ; i < N; ++i)
        {
          if(((1L << i) & dormants) == 0)
          {
            if((1L << lastActive) & activeCombo)
            {
              active.insert(udc._tasks[i]);
              realActiveCode |= (1L << i);
            }
            ++lastActive;
          }
        }

        s._interdicted.clear();
        std::set_difference(s._active.begin(), s._active.end(),
                            active.begin(), active.end(),
                            std::inserter(s._interdicted, s._interdicted.begin()));
         
        stateCount++;

        size_t tau = (1L << 2 * N)  * (budget_ - y);
        tau |= (1L << N) * fcode;
        tau |= realActiveCode;
        
        ActionSharedPtr actionPtr = _policy.at(s);

        double totalRate = 0;
        BOOST_FOREACH(vertex_t v, s._active)
        {
          totalRate += util::rate(v, net_, s, actionPtr);
        }
        double value = _stateEval.evaluate(udc, unet_, *uPtr_, net_, s,actionPtr,totalRate,budget_,fcode,stmap); 
        udc.store(tau, value);
      }
    }
  }
  return stateCount;
}

}}
