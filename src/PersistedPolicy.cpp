#include "PersistedPolicy.hpp"
#include <cstring>
#include <list>

namespace {

const int BUFFER_SIZE = 512;

using namespace ig::core;

std::string udcFileName(const OrderedTaskSet& ts, const std::string& persistDir)
{
  std::stringstream sstr;
  sstr << persistDir << "/";
  BOOST_FOREACH(vertex_t t, ts)
  {
    sstr << t << "_";
  }
  sstr << ".udc";
  return sstr.str();
}

}

namespace ig { namespace core {

Action PersistedPolicy::at(const State& state_) const
{
  OrderedTaskSet theUDC;
  std::set_union(state_._active.begin(), state_._active.end(),
                 state_._dormant.begin(), state_._dormant.end(),
                 std::inserter(theUDC, theUDC.begin()));
  if(theUDC != _currentUDC)
  {
    // the sin of copying is forgiven here for it aint perf critical
    _currentUDC = theUDC;
    std::cout << "Loading a new UDC to cache" << std::endl;
    std::ifstream in(udcFileName(_currentUDC, _persistDir).c_str());

    in.seekg(0, std::ios::end);
    size_t length = in.tellg();
    in.seekg(0, std::ios::beg);
    for(size_t pos = 0; pos < length; pos += 1 +  _net.size())
    {
      Action action;
      StateSharedPtr sPtr(new State);
      readState(in, *sPtr, action);
      //read in the taul value  
      return action;
    }
    std::cout << "Finished loading the new UDC. Continuing." << std::endl;
  }

  return Action();
}

void PersistedPolicy::readState(std::ifstream& input_, State& state_, Action& action_) const
{
  char buf[BUFFER_SIZE];
  size_t bufferLen = _net.size() + 1;
  input_.read(buf, bufferLen);
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
        action_.insert(*vi);
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

PersistedPolicy::PersistedPolicy(const std::string& persistDir_, const Network& net_)
  : _net(net_), _persistDir(persistDir_)
{
}

PersistantStoragePolicy::PersistantStoragePolicy(const std::string& persistDir_, const Network& net_)
  : _net(net_), _persistDir(persistDir_)
{
}

void PersistantStoragePolicy::operator() (const State& state_, const Action& action_)
{
  OrderedTaskSet theUDC;
  std::set_union(state_._active.begin(), state_._active.end(),
                 state_._dormant.begin(), state_._dormant.end(),
                 std::inserter(theUDC, theUDC.begin()));
  if(theUDC != _currentUDC)
  {
    //ok, copying a set across.. not the end of the world and this isn't a performance critical
    //section
    _currentUDC = theUDC;

    if(_out.is_open())
    {
      _out.close();
    }
    _out.open(udcFileName(_currentUDC, _persistDir).c_str());
  }

  char buf[BUFFER_SIZE];
  size_t bufferLen = _net.size() + 1;
  std::memset(buf, 0, bufferLen);
  BOOST_FOREACH(vertex_t t, state_._active)
  {
    buf[t] |= 1;
  }
  BOOST_FOREACH(vertex_t t, action_.asTaskSet())
  {
    buf[t] |= 4;
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
  _out.write(buf, bufferLen);
}

}}
