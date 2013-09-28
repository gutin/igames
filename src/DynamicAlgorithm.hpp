#ifndef DYNAMICALGORITHM_H
#define DYNAMICALGORITHM_H

#include "CommonTypes.hpp"
#include "State.hpp"
#include "Utils.hpp"

#include <boost/progress.hpp>

namespace ig { namespace core {

struct StateTemplate
{
  uvertex_t _udc;
  int _activationCode;

  StateTemplate(uvertex_t udc_=0, int activationCode_=0) :
    _udc(udc_), _activationCode(activationCode_) {}
};

typedef std::map<int, StateTemplate> StateTemplateMap; 

struct StandardEvaluator
{

  static double evaluate(const UDC& udc, const UDCNetwork& unet_, uvertex_t uv_,
                  const Network& net_, 
                  const State& s, const ActionSharedPtr& candidate,
                  double totalRate, size_t budget_,
                  int fcode, StateTemplateMap& stmap);  
};

typedef boost::unordered_map<State, ActionSharedPtr, StateHash> DynamicPolicy;

template<class StateEvaluator = StandardEvaluator>
class DynamicAlgorithm
{
public:
  static void optimalPolicyAndValue(const Network& network_, size_t budget_,
                DynamicPolicy& optimalPolicy_, double& optimalValue_);
  static double optimalValue(const Network& network_, size_t budget_);
private:
  template<class DecisionStoragePolicy>
  static double execute(const Network&,size_t,DecisionStoragePolicy&);

  template<class DecisionStoragePolicy>
  static int solveUDC(vertex_i, UDCNetwork&, const Network&, size_t, DecisionStoragePolicy&);

  static size_t tau(uvertex_t udc_, const UDCNetwork& net_, const State& state, size_t budget_); 
};

struct NoDecisionStorage
{
  void operator() (const State& /*state_*/, const ActionSharedPtr& /* action_ */) {}
};

struct DecisionStorageInMemory
{
  DynamicPolicy& _policy;

  DecisionStorageInMemory(DynamicPolicy& policy_) : _policy(policy_) {}

  void operator() (const State& state_, const ActionSharedPtr& action_)
  {
    _policy[state_] = action_;  
  }
};

template <class SE>
void DynamicAlgorithm<SE>::optimalPolicyAndValue(const Network& network_, size_t budget_,
                DynamicPolicy& optimalPolicy_, double& optimalValue_) 
{
  DecisionStorageInMemory dsm(optimalPolicy_);
  optimalValue_ = execute(network_, budget_, dsm);
}

template <class SE>
double DynamicAlgorithm<SE>::optimalValue(const Network& network_, size_t budget_)
{
  NoDecisionStorage noStorage;
  return execute(network_, budget_, noStorage);
}

template<class SE>
template<class DSP>
double DynamicAlgorithm<SE>::execute(const Network& net_,
                                   size_t budget_,
                                   DSP& storagePolicy_)
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
    int states = solveUDC(uPtr, unet, net_, budget_, storagePolicy_);
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
  int states = solveUDC(startUdc, unet, net_, budget_, storagePolicy_);
  std::cout << "Last one had " << states << " states" << std::endl;
  totalStates += states;
  std::cout << "In total there were " << totalStates << " states" << std::endl;
  State startingState;
  util::startingState(net_, budget_, startingState);
  return unet[*startUdc].value(tau(*startUdc, unet, startingState, budget_));
}

template<class SE>
template<class DSP>
int DynamicAlgorithm<SE>::solveUDC(vertex_i uPtr_, 
                                 UDCNetwork& unet_,
                                 const Network& net_,
                                 size_t budget_,
                                 DSP& storagePolicy_)
{
  StateTemplateMap stmap;
  int stateCount(0);
  UDC& udc = unet_[*uPtr_];
  int maxFCode = (1 << udc.size()) - 2;
  std::cout << "Starting with a maximum finish code of " << maxFCode 
            << " for UDC " << net_.asString(udc._tasks) << std::endl;
  
  size_t N = udc.size();
  for(size_t y = 0; y <= budget_; ++y)
  {
    for(int fcode = maxFCode; fcode >= 0; --fcode)
    {
      State s(y);
      int dormants = ((1 << N) -1) & fcode;

      for(int i = 0; i < N; ++i)
      {
        if(dormants & (1 << i))
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
      int maxActiveCode = (1 << s._active.size()) - 1;
      for(int activeCombo = maxActiveCode; activeCombo >= 0; --activeCombo)
      {
        int realActiveCode = 0;
        OrderedTaskSet active;
        int lastActive = 0;
        for(int i = 0 ; i < N; ++i)
        {
          if(((1 << i) & dormants) == 0)
          {
            if((1 << lastActive) & activeCombo)
            {
              active.insert(udc._tasks[i]);
              realActiveCode |= (1 << i);
            }
            ++lastActive;
          }
        }

        s._interdicted.clear();
        std::set_difference(s._active.begin(), s._active.end(),
                            active.begin(), active.end(),
                            std::inserter(s._interdicted, s._interdicted.begin()));
         
        stateCount++;

        long tau = (1 << 2 * N)  * (budget_ - y);
        tau |= (1 << N) * fcode;
        tau |= realActiveCode;

        double maxVal = 0;
        ActionSharedPtr bestAction;

        TaskList intEligible(active.begin(), active.end());
        for(size_t i = 0; i < (1 << intEligible.size()); ++i)
        {
          if(ig::core::util::numBitsSet(i) > y) continue;
          ActionSharedPtr candidate(new Action); 
          for(int b = 0; b < intEligible.size(); ++b)
          {
            if(((i >> b) & 1) == 1)
            {
              candidate->insert(intEligible[b]);
            }
          }

          if(candidate->size() > y)
            std::cerr << "Invalid action sleected!" << std::endl;

          double totalRate = 0;
          BOOST_FOREACH(vertex_t v, s._active)
          {
            totalRate += util::rate(v, net_, s, candidate);
          }
          
          double val = SE::evaluate(udc, unet_, *uPtr_, net_, 
                                    s, candidate,
                                    totalRate, budget_,
                                    fcode, stmap);  
          if(val > maxVal)
          {
            maxVal = val;
            bestAction = candidate;
          }
        }

        udc.store(tau, maxVal);
        storagePolicy_(s, bestAction);
        //std::cout << "optimal action in state " << s.asString() << " is " << net_.asString(bestAction ? *bestAction : OrderedTaskSet()) 
        //  << " and value is " << maxVal << " and tau = " << tau << std::endl;  
      }
    }
  }
  return stateCount;
}


template<class SE>
size_t DynamicAlgorithm<SE>::tau(uvertex_t udc_, 
    const UDCNetwork& unet_,
    const State& state,
    size_t budget_)
{
  size_t result(0);
  size_t count(0);

  BOOST_FOREACH(vertex_t t, unet_[udc_]._tasks)
  {
    int dig = (1 << count);
    if(state._active.find(t) != state._active.end()  &&
       state._interdicted.find(t) == state._interdicted.end())
    {
      result += dig;
    }
    ++count;
  }

  BOOST_FOREACH(vertex_t t, unet_[udc_]._tasks)
  {
    int dig = (1 << count);
    if(state._dormant.find(t) != state._dormant.end())
    {
      result += dig;
    }
    ++count;
  }

  result += (1 << (2*unet_[udc_].size())) * (budget_ - state._res);
  return result;
}

StateTemplate& nextTemplate(vertex_t u_, const UDC& udc_, uvertex_t uv_,
                            StateTemplateMap& stmap_, const Network& net_,
                            const UDCNetwork& unet_, int fcode_)
{
  int tidx = udc_._activity2UDCIndex[u_];
  int bitnum = (1 << tidx);
  int udcCompCode = fcode_ | bitnum;

  if(stmap_.find(udcCompCode) != stmap_.end())
  {
    return stmap_[udcCompCode];
  }

  size_t N = udc_.size();

  BigInt allFinished(0);
  BigInt eligible = net_._successorbs[u_];
  BigInt relevant(0);

  for(size_t i = 0; i < N; ++i)
  {
    vertex_t t = udc_._tasks[i];
    if((1 << i) & udcCompCode)
    {
      eligible |= net_._successorbs[t];
      allFinished.set(t);
    }
    relevant.set(t);
  }

  BOOST_FOREACH(vertex_t finishedT, udc_._finished)
  {
    allFinished.set(finishedT);
  }

  vertex_i vi, vi_end;
  boost::tie(vi, vi_end) = boost::vertices(net_.graph());
  for(; vi != vi_end; ++vi)
  {
    if(eligible[*vi])
    {
      BigInt predBs = net_._predbs[*vi];
      BigInt tmp = predBs & allFinished;
      if(tmp == predBs)
      {
        relevant.set(*vi);
        relevant &= ~predBs;
      }
    }
  }

  bool found = false;
  uvertex_t theUDC;
  if(udc_._taskBVec == relevant)
  {
    found = true;
    theUDC = uv_;
  }
  else
  {
    oute_i ei, ei_end;
    boost::tie(ei, ei_end) = boost::out_edges(uv_, unet_._ug);
    for(; ei != ei_end; ++ei)
    {
      uvertex_t next = boost::target(*ei, unet_._ug);
      if(unet_._ug[next]._taskBVec == relevant)
      {
        found = true;
        theUDC = next;
        break;
      }
    }
  }
  
  if(!found) 
  {
    std::cerr << "Warning! failed to find UDC. finish code = " << fcode_ 
      << ". comp code = " << udcCompCode << std::endl;
  }
  
  const TaskList& nextUDCTasks = unet_._ug[theUDC]._tasks;
  int nextFinishCode = 0;
  for(size_t i = 0; i < nextUDCTasks.size(); ++i)
  {
    vertex_t nt = nextUDCTasks[i];
    if(allFinished[nt])
    {
      nextFinishCode |= (1 << i);
    }
  }
  StateTemplate result(theUDC, nextFinishCode);
  return (stmap_[udcCompCode] = result);
}

double StandardEvaluator::evaluate(const UDC& udc_,
                                  const UDCNetwork& unet_,
                                  uvertex_t uv_,
                                  const Network& net_, 
                                  const State& state_,
                                  const ActionSharedPtr& candidate_,
                                  double totalRate_, size_t budget_,
                                  int fcode_, StateTemplateMap& stmap_)
{
  double value = 0;
  if(state_._active.find(net_.end()) != state_._active.end() ||
     state_._dormant.find(net_.end()) != state_._dormant.end())
  {
    return value;
  }

  BOOST_FOREACH(vertex_t u, state_._active)
  {
    double nextVal = 0;
    StateTemplate& st = nextTemplate(u, udc_,  uv_,stmap_, net_, unet_, fcode_); 
    const UDC& stUDC = unet_[st._udc]; 
    if(stUDC._taskSet.find(net_.end()) != stUDC._taskSet.end())
    {
      nextVal = 0;
    }
    else
    {
      long tav = (1L << (2* stUDC._tasks.size())) *
                 (budget_ - state_._res + candidate_->size());
      tav |= (1L << stUDC._tasks.size()) * st._activationCode;
      long tmp = (~0) - ((1L << stUDC._tasks.size()) - 1L) + st._activationCode;
      tav |= ~(tmp);

      BOOST_FOREACH(vertex_t t, *candidate_)
      {
        if(stUDC._taskSet.find(t) != stUDC._taskSet.end())
        {
          tav &= ~(1 << stUDC._activity2UDCIndex[t]);
        }
      }
      BOOST_FOREACH(vertex_t t, state_._interdicted)
      {
        if(stUDC._taskSet.find(t) != stUDC._taskSet.end())
        {
          tav &= ~(1 << stUDC._activity2UDCIndex[t]);
        }
      }
      if(stUDC._taskSet.find(u) != stUDC._taskSet.end())
      {
        tav &= ~(1 << stUDC._activity2UDCIndex[u]);
      }
      nextVal = stUDC.value(tav);
    }
    value += (util::rate(u, net_, state_, candidate_) / totalRate_) * nextVal;
  }
  value += 1 / totalRate_; 
  return value;
}

}}

#endif
