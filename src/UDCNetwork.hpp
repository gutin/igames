#ifndef UDCNETWORK_H
#define UDCNETWORK_H

#include "Network.hpp"
#include "CommonTypes.hpp"
#include <boost/unordered_map.hpp>
#include <boost/graph/directed_graph.hpp>

#include <string>

namespace ig { namespace core {

class UDC
{
public:
  TaskList _tasks;
  OrderedTaskSet _taskSet;
  OrderedTaskSet _finished;
  std::vector<float> _v;
  std::vector<long> _indexes;
  std::vector<int> _activity2UDCIndex;
  BigInt _taskBVec;
  size_t _dependCount;

  UDC() : _taskBVec(0), _dependCount(0) {} 
   
  void init(const TaskSet&, const DirectedGraph&);
  size_t rank() const { return _finished.size(); }

  bool singleStepTransition(const UDC& other_, const Network& net_) const;
  
  // We'd like a high to low rank ordering on UDCs (our only use case)
  bool operator<(const UDC& other_) const;

  size_t size() const { return _tasks.size(); }

  void store(size_t tau_, double val_);
  double value(size_t tau_) const;

  void cleanup();
private:
  void rank(const TaskSet&, const DirectedGraph&);
};

class UDCLabelWriter;

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, UDC> UDCGraph;
typedef boost::graph_traits<UDCGraph>::vertex_descriptor uvertex_t;
typedef boost::graph_traits<UDCGraph>::vertex_iterator uvertex_i;
typedef boost::graph_traits<UDCGraph>::out_edge_iterator u_oute_i;
typedef std::vector<uvertex_i> UDCPtrs;

class UDCNetwork
{
public:
  explicit UDCNetwork(const Network&);

  size_t size() const { return boost::num_vertices(_ug); }
  void sortedUDCs(UDCPtrs& ) const;

  UDC& operator[](uvertex_t u_) { return _ug[u_]; }
  const UDC& operator[](uvertex_t u_) const { return _ug[u_]; }
  UDCGraph _ug;
private:
  static void findUDCs(const DirectedGraph& net_, TaskSets& udcs_);
  static void findUDCs(const DirectedGraph& net_, 
                       const TaskList& tasks_,
                       int current_,
                       TaskSet soFar_,
                       TaskSets& udcs_);
  static bool antichain(const DirectedGraph& net_, 
                        const TaskSet& soFar_,
                        vertex_t node_);
  static bool reachable(const DirectedGraph& tc, vertex_t one_, vertex_t other_);


  friend class UDC;
  friend class UDCLabelWriter;
};

class UDCLabelWriter 
{
public:
  UDCLabelWriter(const UDCNetwork& unet_, const Network& net_) : _unet(unet_), _net(net_) {}

  template <class VertexOrEdge>
  void operator()(std::ostream& out_, const VertexOrEdge& v_) const 
  {
    out_ << "[label=\"(" << _net.asString(_unet._ug[v_]._tasks) << ", "
         << _unet._ug[v_].rank() << ")\"]";
  }
private:
  const Network& _net; 
  const UDCNetwork& _unet; 
};

}}

#endif
