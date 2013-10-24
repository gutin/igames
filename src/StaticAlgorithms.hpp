#ifndef DETERMINISTICALGORITHM_H
#define DETERMINISTICALGORITHM_H

#include "CommonTypes.hpp"
#include "UDCNetwork.hpp"
#include "State.hpp"
#include "DynamicAlgorithm.hpp"
#include "DynamicEvaluator.hpp"

#include <vector>

namespace ig { namespace core {

struct StaticPolicy
{
  ActionSharedPtr at(const State&) const;

  StaticPolicy& operator<<(vertex_t u);

  std::string asString() const;
  // the static interdiction pattern 'theta'
  OrderedTaskSet _targets;
};

// this Policy struct will be passed into the efficient dynamic algorithm
// to find all the states in the static problem (same as if budget was 0)
struct StateCollection
{
  void operator() (const State& state_, const ActionSharedPtr& /*unused asp*/)
  {
    if(state_._interdicted.empty())
      _statesAggregated.push_back(state_);
  }
  std::vector<State> _statesAggregated;

};

// Another trick to make sure we don't do extra work while aggregating states
// in the dynamic algorithm
struct NullEvaluator
{
  static double evaluate(const UDC&,const UDCNetwork&,uvertex_t,const Network&,const State&,const ActionSharedPtr&,
                double,size_t,int,StateTemplateMap&) {}
};

double deterministicPolicy(const Network& net_, size_t budget_, StaticPolicy& policy_, bool impunc_ = false);

double staticStochasticPolicy(const Network& net_, size_t budget_, StaticPolicy& policy_);

struct VertexComp 
{
  const Network& net;

  VertexComp(const Network& net) : net(net) {}

  bool operator()(vertex_t one, vertex_t other)
  {
    return net.graph()[one].expNormal() > net.graph()[other].expNormal();  
  }
};

struct ImpuncVertexComp : VertexComp
{
  bool operator()(vertex_t one, vertex_t other)
  {
    const Task& oneTask = net.graph()[one];
    const Task& otherTask = net.graph()[other];
    return (oneTask._probDelaySuccess * oneTask.expNormal()) > (otherTask._probDelaySuccess * otherTask.expNormal());  
  }

  ImpuncVertexComp(const Network& net_) : VertexComp(net_) { std::cout << "Using the IMPUNC vertex comparator" << std::endl; }
};

class ImplementationUncertaintyEvaluator;

template <class Eval>
struct StaticEvalTraits
{
  typedef VertexComp CompType;
  enum { impunc_flag = 0 };
};

template <>
struct StaticEvalTraits<class ImplementationUncertaintyEvaluator>
{
  typedef ImpuncVertexComp CompType;
  enum { impunc_flag = 1 };
};


typedef std::list<StaticPolicy> StaticPolicies;

struct DeterministicDPAlgoTableEntry 
{
  double _value;
  std::list<DeterministicDPAlgoTableEntry*> _successorEntries;
  size_t _budget;
  vertex_t _task;
};

typedef std::vector<std::vector<DeterministicDPAlgoTableEntry> > DeterministicDPAlgoTable; 

double allOptimalDeterministicPolicies(const Network&, size_t, StaticPolicies&);

template <class Evaluator>
double staticStochasticPolicyHeuristic(const Network& net_, size_t budget_, const TaskList& searchSpace_, StaticPolicy& policy_)
{
  double optimalHeuristicValue = std::numeric_limits<double>::min();
  for(size_t bits = 0; bits < (1L << searchSpace_.size()); ++bits)
  {
    if(util::numBitsSet(bits) != budget_)
      continue;
    StaticPolicy policyToEvaluate;
    for(size_t b = 0; b < searchSpace_.size(); ++b)
    {
      if((1L << b) & bits)
      {
        vertex_t t = searchSpace_[b];
        policyToEvaluate << t;
      }
    }
    double value = FastEvaluator<StaticPolicy, Evaluator>(policyToEvaluate).evaluate(net_, budget_);
    if(value > optimalHeuristicValue)
    {
      std::cout << "Improving to " << value << std::endl;
      std::cout << "with: ";
      BOOST_FOREACH(vertex_t c, policyToEvaluate._targets)
      {
        std::cout << c << " ";
      }
      std::cout << std::endl;
      optimalHeuristicValue = value;
      policy_ = policyToEvaluate;  
    }
  }
  return optimalHeuristicValue;
}

template <class Evaluator> 
double staticStochasticPolicyHeuristic(const Network& net_, size_t budget_, StaticPolicy& policy_)
{
  TaskList nLongestRunning;
  vertex_i vi, vi_end;
  boost::tie(vi, vi_end) = boost::vertices(net_.graph());
  std::copy(vi, vi_end, std::inserter(nLongestRunning, nLongestRunning.begin()));
  std::sort(nLongestRunning.begin(), nLongestRunning.end(), typename StaticEvalTraits<Evaluator>::CompType(net_));
  nLongestRunning.resize(budget_ + 1);

  //now add anything from determ to make sure were not worse than that
  StaticPolicy determPolicy;
  deterministicPolicy(net_, budget_,  determPolicy, StaticEvalTraits<Evaluator>::impunc_flag);
  BOOST_FOREACH(vertex_t t, determPolicy._targets)
  {
    if(std::find(nLongestRunning.begin(), nLongestRunning.end(), t) == nLongestRunning.end())
    {
      nLongestRunning.push_back(t);
    }
  }
  return staticStochasticPolicyHeuristic<Evaluator>(net_, budget_, nLongestRunning, policy_);
}

template <class Evaluator> 
double minimalStaticStochasticPolicyHeuristic(const Network& net_, size_t budget_, StaticPolicy& policy_)
{
  StaticPolicies sps;
  allOptimalDeterministicPolicies(net_, budget_, sps);

  StaticPolicy best;
  double optimalVal = std::numeric_limits<double>::min();
  BOOST_FOREACH(const StaticPolicy& sp, sps)
  {
    double value = FastEvaluator<StaticPolicy, Evaluator>(sp).evaluate(net_, budget_);
    if(value > optimalVal)
    {
      best = sp;
      optimalVal = value;
    }
  }
  return optimalVal;
}

void dumpStaticStochasticMILP(const Network& net_, size_t budget_, const std::string& lpFile_);

}}

#endif
