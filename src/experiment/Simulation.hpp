#ifndef SIMULATION_H
#define SIMULATION_H

#include <Network.hpp>
#include <State.hpp>
#include <Utils.hpp>
#include <CommonTypes.hpp>

#include <limits>
#include <ctime>
#include <cmath>

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
};

template <class InterdictionPolicy, class StateVisitor = NullStateVisitor, bool impunc = false>
class Simulation
{
public:
  Simulation(const Network& net_, const InterdictionPolicy& ip_, StateVisitor& visitor_) : 
    _net(net_), _policy(ip_), _visitor(visitor_) { std::srand(time(0)); }

  double run(size_t budget_);

private:
  const Network& _net;
  const InterdictionPolicy& _policy;
  StateVisitor& _visitor;
};

template <class IP, class SV, bool impunc>
double Simulation<IP, SV, impunc>::run(size_t buget_)
{
  StateSharedPtr statePtr(new State);
  ig::core::util::startingState(_net, buget_, *statePtr);
  OrderedTaskSet finished;
  double makespan = 0;

  while(!ig::core::util::isTerminalState(_net, *statePtr))
  {
    // Get the action
    ActionSharedPtr actionPtr = _policy.at(*statePtr);
    
    // Do something with this information
    _visitor.visit(statePtr, actionPtr);

    // Sample random task finish times to see which task finishes first
    double minDuration = std::numeric_limits<double>::max();
    vertex_t finishing;
    BOOST_FOREACH(vertex_t t, statePtr->_active)
    {
      double tDuration = 0;
      if(actionPtr->find(t) != actionPtr->end() ||
         statePtr->_interdicted.find(t) != statePtr->_interdicted.end())
      {
        if(!impunc || 
           ( impunc && (std::rand() / double(RAND_MAX)) < _net.graph()[t]._probDelaySuccess) )
        {
          //roll a dice on whether interdiction succeeded
          tDuration = genExponential(_net.graph()[t]._delta);
        }
        else
        {
          tDuration = genExponential(_net.graph()[t]._nu);
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
    statePtr = ig::core::util::nextState(_net, *statePtr, actionPtr, finished, finishing);
  }
  return makespan;
}

}}
#endif
