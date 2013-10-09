#ifndef COMMONTYPES_H
#define COMMONTYPES_H

#include <Task.hpp>

#include <boost/version.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#if (BOOST_VERSION >= 104900)
#warning "Using a bizarre workaround for transitive closure. \
http://boost.2283326.n4.nabble.com/Problem-in-using-Transitive-closure-on-Linux-td2573042.html"
#include <boost/graph/vector_as_graph.hpp>
#endif
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_archetypes.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#define TASK_ATTR(task, attr) net_.graph()[(task)].attr

#include <iostream>
#include <vector>
#include <bitset>

namespace ig { namespace core {
  class State;

  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Task> DirectedGraph;
  typedef boost::graph_traits<DirectedGraph>::vertex_descriptor vertex_t;
  typedef boost::graph_traits<DirectedGraph>::vertex_iterator vertex_i;
  typedef boost::graph_traits<DirectedGraph>::adjacency_iterator adj_i;
  typedef boost::graph_traits<DirectedGraph>::out_edge_iterator oute_i;
  typedef boost::graph_traits<DirectedGraph>::in_edge_iterator ine_i;

  typedef std::bitset<128> BigInt;

  typedef boost::unordered_set<vertex_t> TaskSet;
  typedef std::set<vertex_t> OrderedTaskSet;

  template <class TaskCollection>
  struct TaskSetHash : std::unary_function<TaskCollection, size_t>
  {
    size_t operator() (const TaskCollection& ts_) const 
    {
      size_t h=0;
      BOOST_FOREACH(const vertex_t& v, ts_)
      {
        h += boost::hash_value(v);
      }
      return h;
    }
  };

  typedef boost::unordered_set<TaskSet, TaskSetHash<TaskSet> > TaskSets;
  
  typedef std::vector<vertex_t> TaskList; 
}}

#endif
