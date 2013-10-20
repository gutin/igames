#ifndef DETERMINISTICALGORITHM_H
#define DETERMINISTICALGORITHM_H

#include "CommonTypes.hpp"
#include "UDCNetwork.hpp"
#include "State.hpp"
#include "DynamicAlgorithm.hpp"

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

double staticStochasticPolicyHeuristic(const Network& net_, size_t budget_, const TaskList& searchSpace_, StaticPolicy& policy_);
double staticStochasticPolicyHeuristic(const Network& net_, size_t budget_, StaticPolicy& policy_);

void dumpStaticStochasticMILP(const Network& net_, size_t budget_, const std::string& lpFile_);

}}

#endif
