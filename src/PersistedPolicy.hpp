#ifndef PERSISTEDPOLICY_H
#define PERSISTEDPOLICY_H

#include "CommonTypes.hpp"
#include "Network.hpp"
#include "State.hpp"
#include "Action.hpp"

#include <string>
#include <fstream>

namespace ig { namespace core {

class PersistedPolicy
{
public:
  Action at(const State& state_) const;

  PersistedPolicy(const std::string& persistDir_, const Network& net_);

private:
  void readState(std::ifstream&, State&, Action&) const;
  
  const std::string _persistDir;
  const Network& _net;
  mutable OrderedTaskSet _currentUDC;

  std::vector<size_t> _indices;
  std::vector<Action> _actions;
};

class PersistantStoragePolicy
{
public:
  PersistantStoragePolicy(const std::string& persistDir_, const Network& net_);

  void operator() (const State& state_, const Action& actionPtr_);
private:
  std::string _persistDir;
  std::ofstream _out;
  const Network& _net;

  OrderedTaskSet _currentUDC;
};

}}

#endif
