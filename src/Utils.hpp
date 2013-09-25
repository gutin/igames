#ifndef UTILS_H
#define UTILS_H

#include "CommonTypes.hpp"
#include "Network.hpp"
#include "State.hpp"
#include <algorithm>

namespace ig { namespace core { namespace util {

inline size_t numBitsSet(int i)
{
  i = i - ((i >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
  return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

inline double rate(vertex_t t, const Network& net_, const State& state_, const ActionSharedPtr& actionPtr_)
{
  double delayedRate = net_.graph()[t]._delta;
  double normalRate = net_.graph()[t]._nu;
  if(actionPtr_->find(t) != actionPtr_->end() || state_._interdicted.find(t) != state_._interdicted.end())
    return net_.graph()[t]._delta;
  return net_.graph()[t]._nu;
}

// because STL doesn't have a proper binary search function??!
template <class Iter, class Type>
Iter binary_search(Iter begin_, Iter end_, const Type& val_)
{
  Iter end = end_;
  while(begin_ < end_)
  {
    Iter mid = begin_ + std::distance(begin_, end_)/2;
    if(*mid == val_)
      return mid;
    if(*mid < val_)
      end_ = mid;
    else
      begin_ = mid + 1;
  }
  return end;
}

inline void startingState(const Network& net_, size_t budget_, State& startingState) 
{
  oute_i ei, ei_end;
  boost::tie(ei, ei_end) = boost::out_edges(net_.start(), net_.graph());
  for(;ei != ei_end; ++ei)
  {
    vertex_t successor = boost::target(*ei, net_.graph());
    startingState._active.insert(successor);
  }
  startingState._res = budget_;
}

}}}

#endif
