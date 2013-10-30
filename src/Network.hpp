#ifndef NETWORK_H
#define NETWORK_H

#include "CommonTypes.hpp"

#include <map>

namespace ig { namespace core {

class Network
{
public:
  Network();
  double orderStrength() const;
  const DirectedGraph& graph() const { return _g; }

  bool import(const std::string& file_, bool delaysFromFile_ = false);

  vertex_t start() const;
  vertex_t end() const;

  bool isStart(vertex_t v_) const { return _start == v_; }
  bool isEnd(vertex_t v_) const { return _end == v_; }

  void initSecondary();
  size_t size() const;

  std::vector<BigInt> _successorbs;
  std::vector<BigInt> _predbs;

  void swapDurations(std::vector<double>&);

  void exportDot(const std::string& dotfile_) const;
private:
  DirectedGraph _g;
  vertex_t _start, _end;

  vertex_i findFromIndex(int i_) const;
  vertex_t add(double,double,double);
  void connect(const vertex_t&, const vertex_t&);
};

}}

#endif
