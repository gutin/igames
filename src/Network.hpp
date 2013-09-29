#ifndef NETWORK_H
#define NETWORK_H

#include "CommonTypes.hpp"

#include <map>

namespace ig { namespace core {

class Network
{
public:
  Network();
  void connect(const vertex_t&, const vertex_t&);
  vertex_t add(const Task&);
  double orderStrength() const;
  const DirectedGraph& graph() const { return _g; }

  bool import(const std::string& file_, bool delaysFromFile_ = false);

  vertex_t start() const;
  vertex_t end() const;

  bool isStart(vertex_t v_) const { return _start == v_; }
  bool isEnd(vertex_t v_) const { return _end == v_; }

  template <class TaskCollection>
  std::string asString(const TaskCollection& ts_) const
  {
    std::stringstream stream;
    stream << "[ ";
    BOOST_FOREACH(vertex_t v, ts_)
    {
      stream << _g[v].index() << " ";
    }
    stream << "]";
    return stream.str();
  }

  void initSecondary();
  size_t size() const;

  std::vector<BigInt> _successorbs;
  std::vector<BigInt> _predbs;
private:
  DirectedGraph _g;
  vertex_t _start, _end;

  vertex_i findFromIndex(int i_) const;
};

}}

#endif
