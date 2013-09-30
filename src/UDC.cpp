#include "UDC.hpp"

namespace ig { namespace core {

bool UDC::operator<(const UDC& other_) const
{
  return rank() > other_.rank();
}

void UDC::cleanup()
{
  _tasks.clear();
  _taskSet.clear();
  _finished.clear();
  _activity2UDCIndex.clear();
}

void UDC::init(const TaskSet& tasks_, const DirectedGraph& tcg_)
{
  _tasks = TaskList(tasks_.begin(), tasks_.end()); 
  std::sort(_tasks.begin(), _tasks.end());
  _taskSet = OrderedTaskSet(tasks_.begin(), tasks_.end());
  _activity2UDCIndex.resize(boost::num_vertices(tcg_));
  BOOST_FOREACH(vertex_t t, tasks_)
  {
    _activity2UDCIndex[t] = 
      std::distance(_tasks.begin(), std::find(_tasks.begin(), _tasks.end(), t));
    _taskBVec.set(t);
  }
  rank(tasks_, tcg_);
}

void UDC::rank(const TaskSet& tasks_, const DirectedGraph& tcg_)
{
  vertex_i vi, vi_end;
  boost::tie(vi, vi_end) = boost::vertices(tcg_);
  TaskSet others;
  for(;vi != vi_end;++vi)
  {
    if(tasks_.find(*vi) == tasks_.end())
    {
      others.insert(*vi);
    }
  }

  BOOST_FOREACH(const vertex_t& o, others)
  {
    bool include = false;
    BOOST_FOREACH(const vertex_t& t, tasks_)
    {
      if(boost::edge(o, t, tcg_).second)
      {
        include = true;
        break;
      }
    }
    if(include)
    {
      _finished.insert(o);
    }
  }
}

bool UDC::singleStepTransition(const UDC& other_, const Network& net_) const
{
  if(other_.rank() <= rank())
    return false;
  OrderedTaskSet difference;
  std::set_difference(other_._taskSet.begin(), other_._taskSet.end(),
                      _taskSet.begin(), _taskSet.end(), 
                      std::inserter(difference, difference.begin()));
  if(difference.empty())
    return false;
  OrderedTaskSet intersection;
  std::set_union(_finished.begin(), _finished.end(),
                      _taskSet.begin(), _taskSet.end(), 
                      std::inserter(intersection, intersection.begin()));
  
  OrderedTaskSet preds(intersection);
  BOOST_FOREACH(vertex_t cand, difference)
  {
    ine_i ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(cand, net_.graph());
    OrderedTaskSet predecessors;
    for(;ei != ei_end; ++ei)
    {
      vertex_t u = boost::source(*ei, net_.graph());
      if(preds.find(u) == preds.end())
        return false;
      predecessors.insert(u);
    }
    OrderedTaskSet intermediateIntersection;
    std::set_intersection(intersection.begin(), intersection.end(), 
                          predecessors.begin(), predecessors.end(),
                          std::inserter(intermediateIntersection, intermediateIntersection.begin()));
    intersection = intermediateIntersection;
  }
  return !intersection.empty();
}

}}
