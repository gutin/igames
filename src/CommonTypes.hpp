#ifndef COMMONTYPES_H
#define COMMONTYPES_H

#include "Task.hpp"

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

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
  typedef OrderedTaskSet Action;
  typedef boost::shared_ptr<Action> ActionSharedPtr;

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
