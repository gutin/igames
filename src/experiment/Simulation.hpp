#ifndef SIMULATION_H
#define SIMULATION_H

#include <Network.hpp>
#include <State.hpp>
#include <Utils.hpp>
#include <CommonTypes.hpp>

#include <limits>
#include <ctime>
#include <cmath>
#include <iterator>

namespace {

// trivial to implement my own function.. so why not
double genExponential(double lambda_)
{
  double u = std::rand() / double(RAND_MAX);
  return -std::log(u) / lambda_;    
}

}

using namespace ig::core;

namespace ig { namespace experiment {

struct NullStateVisitor 
{
  void visit(const StateSharedPtr& state, const ActionSharedPtr& actionPtr_) const
  {}

  void recordCompletion(vertex_t,double)
  {}
};

template <class InterdictionPolicy, class StateVisitor = NullStateVisitor, bool impunc = false, bool logInfo = false>
class Simulation
{
public:
  Simulation(const Network& net_, const InterdictionPolicy& ip_, StateVisitor& visitor_) : 
    _net(net_), _policy(ip_), _visitor(visitor_) { std::srand(time(0)); }

  double run(size_t budget_);

  StateVisitor& _visitor;
private:
  const Network& _net;
  const InterdictionPolicy& _policy;
};

template <class IP, class SV, bool impunc, bool logInfo>
double Simulation<IP, SV, impunc, logInfo>::run(size_t buget_)
{
  StateSharedPtr statePtr(new State);
  ig::core::util::startingState(_net, buget_, *statePtr);
  OrderedTaskSet finished;
  double makespan = 0;

  std::vector<double> taskStartTimes(_net.size(), 0);
  while(!ig::core::util::isTerminalState(_net, *statePtr))
  {
    // Get the action
    ActionSharedPtr actionPtr = _policy.at(*statePtr);
    if(logInfo)
    {
      std::cout << "Action in state " << statePtr->asString() << " is [";
      std::copy(actionPtr->begin(), actionPtr->end(),
                std::ostream_iterator<vertex_t>(std::cout, " "));
      std::cout << "]" << std::endl;
    }
    
    // Do something with this information
    _visitor.visit(statePtr, actionPtr);

    // Sample random task finish times to see which task finishes first
    double minDuration = std::numeric_limits<double>::max();
    vertex_t finishing;
   
    ActionSharedPtr effectiveActionPtr = impunc ? ActionSharedPtr(new Action(*actionPtr)) : actionPtr;
    BOOST_FOREACH(vertex_t t, statePtr->_active)
    {
      double tDuration = 0;
      if(statePtr->_interdicted.find(t) != statePtr->_interdicted.end())
      {
        tDuration = genExponential(_net.graph()[t]._delta);
      }
      else if(actionPtr->find(t) != actionPtr->end())
      {
        if(!impunc || 
           ( impunc && (double(std::rand()) / double(RAND_MAX)) < _net.graph()[t]._probDelaySuccess) )
        {
          //roll a dice on whether interdiction succeeded
          tDuration = genExponential(_net.graph()[t]._delta);
        }
        else
        {
          tDuration = genExponential(_net.graph()[t]._nu);
          effectiveActionPtr->erase(t);
        }
      }
      else
      {
        tDuration = genExponential(_net.graph()[t]._nu);
      }
      if(tDuration < minDuration)
      {
        minDuration = tDuration;
        finishing = t;
      }
    }
    finished.insert(finishing);
    makespan += minDuration;
    BOOST_FOREACH(vertex_t t, statePtr->_active)
    {
      taskStartTimes[t] += minDuration; 
    }
    _visitor.recordCompletion(finishing, taskStartTimes[finishing]);
    if(logInfo)
    {
      std::cout << "Task " << finishing << " completed after " << minDuration << " weeks." << std::endl;
    }
    statePtr = ig::core::util::nextState(_net, *statePtr, effectiveActionPtr, finished, finishing, actionPtr->size());
  }
  return makespan;
}

}}
#endif
