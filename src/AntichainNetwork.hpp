#ifndef UDCNETWORK_H
#define UDCNETWORK_H

#include "Network.hpp"
#include "CommonTypes.hpp"
#include <boost/unordered_map.hpp>
#include <boost/graph/directed_graph.hpp>

#include <string>

namespace ig { namespace core {

template <class Antichain>
class AntichainLabelWriter;

template <class Antichain>
struct AntichainGraph
{
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Antichain> type; 
};

template <class Antichain>
struct acvertex_t
{
  typedef typename boost::graph_traits<typename AntichainGraph<Antichain>::type>::vertex_descriptor type;
};

template <class Antichain>
struct acvertex_i
{
  typedef typename boost::graph_traits<typename AntichainGraph<Antichain>::type>::vertex_iterator type;
};

template <class Antichain>
struct ac_oute_i
{
  typedef typename boost::graph_traits<typename AntichainGraph<Antichain>::type>::out_edge_iterator type;
};

template <class Antichain>
struct AntichainPtrs
{
  typedef typename std::vector<typename acvertex_i<Antichain>::type> type;
};

template <class Antichain>
class AntichainNetwork
{
public:
  explicit AntichainNetwork(const Network&);

  size_t size() const { return boost::num_vertices(_ug); }
  void sortedUDCs(typename AntichainPtrs<Antichain>::type& ) const;

  Antichain& operator[](typename acvertex_t<Antichain>::type u_) { return _ug[u_]; }
  const Antichain& operator[](typename acvertex_t<Antichain>::type u_) const { return _ug[u_]; }
  
  typename AntichainGraph<Antichain>::type _ug; 
  static bool reachable(const DirectedGraph& tc, vertex_t one_, vertex_t other_);

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
};

template <class Antichain>
class AntichainLabelWriter 
{
public:
  AntichainLabelWriter(const AntichainNetwork<Antichain>& unet_, const Network& net_) :
    _unet(unet_), _net(net_) {}

  template <class VertexOrEdge>
  void operator()(std::ostream& out_, const VertexOrEdge& v_) const 
  {
    out_ << "[label=\"(" << _net.asString(_unet._ug[v_]._tasks) << ", "
         << _unet._ug[v_].rank() << ")\"]";
  }
private:
  const Network& _net; 
  const AntichainNetwork<Antichain>& _unet; 
};

}}

#include "AntichainNetwork.cpp"

#endif
