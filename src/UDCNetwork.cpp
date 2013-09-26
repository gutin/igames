#include "UDCNetwork.hpp"
#include "Utils.hpp"

#include <boost/graph/transitive_closure.hpp>
#include <boost/bind.hpp>
#include <boost/progress.hpp>

#include <iostream>
#include <algorithm>

using namespace std;
using namespace boost;

namespace 
{
  struct taskComparator 
  {
    taskComparator(const ig::core::DirectedGraph& g_) : _g(g_) {}

    bool operator() (const ig::core::vertex_t& u_, const ig::core::vertex_t& v_) const
    {
      return _g[u_].index() < _g[v_].index();
    }

    const ig::core::DirectedGraph& _g;
  };

  using ig::core::UDC;
  using ig::core::vertex_t;

  struct containsUDC
  {
    const UDC* udc_;
    containsUDC(const UDC* udc) : udc_(udc) {}

    bool operator() (const vertex_t v_) const
    {
      return find(udc_->_tasks.begin(), udc_->_tasks.end(), v_) != udc_->_tasks.end();
    }
  };
}

namespace ig { namespace core {

UDCNetwork::UDCNetwork(const Network& net_)
{
  DirectedGraph tcg;
  boost::transitive_closure(net_.graph(), tcg);

  TaskSets udcs;
  findUDCs(tcg, udcs);
  cout << "Generated " << udcs.size() << " cuts" << endl;    
  
  BOOST_FOREACH(const TaskSet& ts, udcs)
  {
    cout << net_.asString(ts) << endl;
    if(std::find_if(ts.begin(), ts.end(), boost::bind(&Network::isStart, net_, _1)) != ts.end())
    {
      continue;
    }
    uvertex_t udc = boost::add_vertex(_ug);
    _ug[udc].init(ts, tcg);
  }
  
  uvertex_i vi, vi_end; 
  boost::tie(vi, vi_end) = boost::vertices(_ug);
  uvertex_i start_i = vi;
  for(;vi != vi_end; ++vi)
  {
    uvertex_i vii = start_i;
    for(;vii != vi_end;++vii)
    {
      if(vi == vii) continue;
      if(_ug[*vi].singleStepTransition(_ug[*vii], net_))
      {
        boost::add_edge(*vi, *vii, _ug);
        ++(_ug[*vii]._dependCount);
      }
    }
  }
  cout << "Constructed UDC network" << endl;
}

void UDCNetwork::findUDCs(const DirectedGraph& n_, TaskSets& udcs_)
{
  boost::progress_timer timer;
  vertex_i vi, vi_end;
  boost::tie(vi, vi_end) = boost::vertices(n_);
  TaskList tasks(vi, vi_end); 
  
  std::sort(tasks.begin(), tasks.end(), taskComparator(n_));
  for(size_t i = 0; i < tasks.size(); ++i)
  {
    TaskSets tsets;
    findUDCs(n_, tasks, i, TaskSet(), tsets);
    std::copy(tsets.begin(), tsets.end(), std::inserter(udcs_, udcs_.end()));
  }
}

void UDCNetwork::findUDCs(const DirectedGraph& net_, 
                          const TaskList& tasks_,
                          int current_,
                          TaskSet soFar_,
                          TaskSets& udcs_)
{
  vertex_t node = tasks_[current_];
  if(antichain(net_, soFar_, node))
  {
    bool maximal=true;
    TaskSet bigger = soFar_;
    bigger.insert(node);
    for(size_t i = 0; i < tasks_.size(); ++i)
    {
      if(bigger.find(tasks_[i]) != bigger.end())
      {
        continue;
      }
      if(antichain(net_, bigger, tasks_[i]))
      {
        maximal = false;
        break;
      }
    }
    if(maximal)
    {
      udcs_.insert(bigger);
    }
    else
    {
      for(size_t i = current_ + 1; i < tasks_.size(); ++i)
      {
        TaskSets subUdcs;
        findUDCs(net_, tasks_, i, bigger, subUdcs); 
        std::copy(subUdcs.begin(), subUdcs.end(), std::inserter(udcs_, udcs_.end()));
      }
    }
  }
}

bool UDCNetwork::antichain(const DirectedGraph& net_, 
                      const TaskSet& soFar_,
                      vertex_t node_)
{
  if(soFar_.find(node_) != soFar_.end())
  {
    return true;
  }

  BOOST_FOREACH(const vertex_t& v, soFar_)
  {
    if(reachable(net_, v, node_)
       || reachable(net_, node_, v))
    {
      return false;
    }
  }
  return true;
}

bool UDCNetwork::reachable(const DirectedGraph& tc_, vertex_t one_, vertex_t other_)
{
  return boost::edge(one_, other_, tc_).second;
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
      if(UDCNetwork::reachable(tcg_, o, t))
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

struct UDCSortComparison
{
  UDCSortComparison(const UDCNetwork& unet_) : _unet(unet_) {}
  
  bool operator() (uvertex_i i, uvertex_i j)
  {
    return _unet[*i] < _unet[*j];
  }
  const UDCNetwork& _unet;
};

void UDCNetwork::sortedUDCs(UDCPtrs& ptrs_) const
{
  ptrs_.reserve(size());

  uvertex_i uvi, uvi_end;
  boost::tie(uvi, uvi_end) = boost::vertices(_ug);
  for(;uvi != uvi_end; ++uvi)
  {
    cout << "pushing back " << *uvi << endl;
    ptrs_.push_back(uvi);      
  }
  std::sort(ptrs_.begin(), ptrs_.end(), UDCSortComparison(*this));
}

bool UDC::operator<(const UDC& other_) const
{
  return rank() > other_.rank();
}


void UDC::store(size_t tau_, double val_)
{
  // Can we optimise by pre-sizing the vectors?
  _v.push_back(val_);
  _indexes.push_back(tau_);
}

double UDC::value(size_t tav_) const
{
  std::vector<long>::const_iterator indexIter = util::binary_search(_indexes.begin(),
                                                                    _indexes.end(),
                                                                    tav_);
  if(indexIter == _indexes.end())
  {
    std::cerr << "Error! Failed to value for " << tav_ << std::endl;
    return 0;
  }
  return _v[std::distance(_indexes.begin(), indexIter)];
}

void UDC::cleanup()
{
  _tasks.clear();
  _taskSet.clear();
  _finished.clear();
  _indexes.clear();
  _v.clear();
  _activity2UDCIndex.clear();
}

}}
