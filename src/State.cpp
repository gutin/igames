#include "State.hpp"

namespace ig { namespace core {

std::string State::asString() const
{
  std::stringstream sstr;
  sstr << "{A=[";
  OrderedTaskSet::const_iterator ti = _active.begin();
  while(ti != _active.end())
  {
    sstr << *ti; 
    if(_interdicted.find(*ti) != _interdicted.end())
      sstr << "*";
    ++ti;
    if(ti == _active.end())
      break;
    sstr << ",";
  }
  sstr << "]:F=[";
  ti = _dormant.begin();
  while(ti != _dormant.end())
  {
    sstr << *ti; 
    ++ti;
    if(ti == _dormant.end()) 
      break;
    sstr << ",";
  }
  sstr << "]:r=" << _res << "}";
  return sstr.str();
}


bool State::operator==(const State& other_) const
{
  return _active == other_._active && _interdicted == other_._interdicted 
        && _dormant == other_._dormant && _res == other_._res;
}

void State::addInterdictedSet(const OrderedTaskSet& interdicted_)
{
  BOOST_FOREACH(vertex_t u, interdicted_)
  {
    _interdicted.insert(u);
  }
}

}}
