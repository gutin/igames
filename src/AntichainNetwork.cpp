#include "Utils.hpp"

#include <boost/graph/transitive_closure.hpp>
#include <boost/bind.hpp>
#include <boost/progress.hpp>

#include <iostream>
#include <algorithm>
#include <cstdlib>

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
}

namespace ig { namespace core {

template <class AC>
AntichainNetwork<AC>::AntichainNetwork(const Network& net_)
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
    typename acvertex_t<AC>::type udc = boost::add_vertex(_ug);
    _ug[udc].init(ts, tcg);
  }
  
  typename acvertex_i<AC>::type vi, vi_end; 
  boost::tie(vi, vi_end) = boost::vertices(_ug);
  typename acvertex_i<AC>::type start_i = vi;
  for(;vi != vi_end; ++vi)
  {
    typename acvertex_i<AC>::type vii = start_i;
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

template <class AC>
void AntichainNetwork<AC>::findUDCs(const DirectedGraph& n_, TaskSets& udcs_)
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

template <class AC>
void AntichainNetwork<AC>::findUDCs(const DirectedGraph& net_, 
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

template <class AC>
bool AntichainNetwork<AC>::antichain(const DirectedGraph& net_, 
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

template <class AC>
bool AntichainNetwork<AC>::reachable(const DirectedGraph& tc_, vertex_t one_, vertex_t other_)
{
  return boost::edge(one_, other_, tc_).second;
}


template <class AC>
struct UDCSortComparison
{
  UDCSortComparison(const AntichainNetwork<AC>& unet_) : _unet(unet_) {}
  
  bool operator() (typename acvertex_i<AC>::type i, typename acvertex_i<AC>::type j)
  {
    return _unet[*i] < _unet[*j];
  }
  const AntichainNetwork<AC>& _unet;
};

template <class AC>
void AntichainNetwork<AC>::sortedUDCs(typename AntichainPtrs<AC>::type& ptrs_) const
{
  ptrs_.reserve(size());

  typename acvertex_i<AC>::type uvi, uvi_end;
  boost::tie(uvi, uvi_end) = boost::vertices(_ug);
  for(;uvi != uvi_end; ++uvi)
  {
    cout << "pushing back " << *uvi << endl;
    ptrs_.push_back(uvi);      
  }
  std::sort(ptrs_.begin(), ptrs_.end(), UDCSortComparison<AC>(*this));
}

}}
