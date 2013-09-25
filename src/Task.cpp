#include "Task.hpp"

namespace ig { namespace core {

bool Task::operator==(const Task& other_) const 
{
  return _index == other_._index; 
}

void Task::setFrom(const Task& other_)
{
  _index = other_._index;
  _nu = other_._nu;
  _delta = other_._delta;
}

}}


