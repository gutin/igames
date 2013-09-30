#ifndef UDC_H
#define UDC_H

#include "AntichainNetwork.hpp"


namespace ig { namespace core {

class UDC
{
public:
  TaskList _tasks;
  OrderedTaskSet _taskSet;
  OrderedTaskSet _finished;
  std::vector<float> _v;
  std::vector<size_t> _indexes;
  std::vector<int> _activity2UDCIndex;
  BigInt _taskBVec;
  size_t _dependCount;

  UDC() : _dependCount(0) {} 
   
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

typedef AntichainNetwork<UDC> UDCNetwork;
typedef AntichainPtrs<UDC>::type UDCPtrs;
typedef acvertex_t<UDC>::type uvertex_t;
typedef acvertex_i<UDC>::type uvertex_i;
typedef ac_oute_i<UDC>::type u_oute_i;

}}

namespace {

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

#endif
