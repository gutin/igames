#ifndef STATE_H
#define STATE_H

#include "CommonTypes.hpp"
#include <string>
#include <sstream>
#include <Network.hpp>

namespace ig { namespace core {

struct State
{
  // sets of active, idle and interdicted tasks
  OrderedTaskSet _active, _interdicted, _dormant;

  // residual budget in that state
  size_t _res; 

  bool operator==(const State& other_) const;

  explicit State(size_t res_ = 0) : _res(res_) {}

  std::string asString() const;

  void addInterdictedSet(const OrderedTaskSet& interdicted_);
  void removeDormantSet(const Network& net_, ine_i begin_, ine_i end_);
};

typedef boost::shared_ptr<State> StateSharedPtr;

struct StateHash : std::unary_function<State, size_t>
{
  size_t operator() (const State& state_) const 
  {
    const size_t prime = 31;
    size_t h=1;
    h = h*prime + TaskSetHash<OrderedTaskSet>()(state_._active);
    h = h*prime + TaskSetHash<OrderedTaskSet>()(state_._interdicted);
    h = h*prime + TaskSetHash<OrderedTaskSet>()(state_._dormant);
    h = h*prime + state_._res;
    return h;
  }
};

struct StateSharedPtrHash : std::unary_function<StateSharedPtr, size_t>
{
  size_t operator() (const StateSharedPtr& statePtr_) const 
  {
    if(!statePtr_) return 0;
    return StateHash()(*statePtr_);
  }
};

struct StateSharedPtrEquivalence : std::binary_function<StateSharedPtr, StateSharedPtr, bool>
{
  //Two statesharedptr are considered equivalent if their underlying states are
  bool operator() (const StateSharedPtr& one_, const StateSharedPtr& other_) const
  {
    if(!one_) return !other_;
    if(!other_) return !one_;
    return *one_ == *other_;   
  }
};

template <class Value>
struct StateSharedPtrMap
{
  typedef boost::unordered_map<StateSharedPtr, Value, StateSharedPtrHash, StateSharedPtrEquivalence> type;
};

}}
#endif
