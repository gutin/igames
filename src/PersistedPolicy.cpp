#include "PersistedPolicy.hpp"
#include <cstring>

namespace {
  const int BUFFER_SIZE = 512;
}

namespace ig { namespace core {

ActionSharedPtr PersistedPolicy::at(const State& state_) const
{
  ActionSharedPtr ret(new Action);
  _input.seekg(0, std::ios::end);
  size_t length = _input.tellg();
  _input.seekg(0, std::ios::beg);
  for(size_t pos = 0; pos < length; pos += 1 +  _net.size())
  {
    State s;
    readState(s, ret);
    if(s == state_)
    {
      return ret;
    }
    ret->clear();
  }
  std::cout << "Failed to find decision in state " << state_.asString() << std::endl;
  abort();
}

void PersistedPolicy::readState(State& state_, ActionSharedPtr& actionPtr_) const
{
  char buf[BUFFER_SIZE];
  size_t bufferLen = _net.size() + 1;
  _input.read(buf, bufferLen);
  vertex_i vi, vi_end;
  for(boost::tie(vi, vi_end) = boost::vertices(_net.graph()); vi != vi_end; ++vi)
  {
    switch(buf[*vi])
    {
      case 1:
        state_._active.insert(*vi);
        break;
      case 2:
        state_._interdicted.insert(*vi);
        state_._active.insert(*vi);
        break;
      case 3:
        state_._dormant.insert(*vi);
        break;
      case 5:
        state_._active.insert(*vi);
        actionPtr_->insert(*vi);
        break;
      case 0:
        break;
      default:
        std::cout << "Unknown byte " << static_cast<int>(buf[*vi]) << " activity " << *vi << std::endl;
        abort();
    }
  }
  state_._res = buf[_net.size()];
}

PersistedPolicy::PersistedPolicy(const std::string& file_, const Network& net_)
  : _net(net_)
{
  _input.open(file_.c_str(), std::ios::binary | std::ios::in);
}

PersistedPolicy::~PersistedPolicy()
{
  _input.close();
}

PersistantStoragePolicy::PersistantStoragePolicy(const std::string& file_, const Network& net_)
  : _net(net_)
{
  _output.open(file_.c_str(), std::ios::binary | std::ios::out);
}

void PersistantStoragePolicy::operator() (const State& state_, const ActionSharedPtr& actionPtr_)
{
  char buf[BUFFER_SIZE];
  size_t bufferLen = _net.size() + 1;
  std::memset(buf, 0, bufferLen);
  BOOST_FOREACH(vertex_t t, state_._active)
  {
    buf[t] |= 1;
  }
  if(actionPtr_)
  {
    BOOST_FOREACH(vertex_t t, *actionPtr_)
    {
      buf[t] |= 4;
    }
  }
  BOOST_FOREACH(vertex_t t, state_._interdicted)
  {
    buf[t] = 2;
  }
  BOOST_FOREACH(vertex_t t, state_._dormant)
  {
    buf[t] = 3;
  }
  buf[_net.size()] = state_._res;
  _output.write(buf, bufferLen);
}

PersistantStoragePolicy::~PersistantStoragePolicy()
{
  _output.close();
}

}}
