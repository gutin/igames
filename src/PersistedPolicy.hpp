#ifndef PERSISTEDPOLICY_H
#define PERSISTEDPOLICY_H

#include "CommonTypes.hpp"
#include "Network.hpp"
#include "State.hpp"

#include <string>
#include <fstream>

namespace ig { namespace core {

class PersistedPolicy
{
public:
  ActionSharedPtr at(const State& state_) const;

  PersistedPolicy(const std::string& file_, const Network& net_);
  ~PersistedPolicy();

private:
  void readState(State&, ActionSharedPtr&) const;
  
  mutable std::ifstream _input;
  const Network& _net;
};

class PersistantStoragePolicy
{
public:
  PersistantStoragePolicy(const std::string& file_, const Network& net_);
  ~PersistantStoragePolicy();

  void operator() (const State& state_, const ActionSharedPtr& actionPtr_);
private:
  std::ofstream _output;
  const Network& _net;
};

}}

#endif
