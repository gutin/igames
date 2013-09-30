#include "UDC.hpp"

#ifndef DYNAMICALGOUDC_H
#define DYNAMICALGOUDC_H

namespace ig { namespace core {

class DynamicAlgoUDC : public UDC
{
public:
  std::vector<float> _v;
  std::vector<size_t> _indexes;

  void store(size_t tau_, double val_);
  double value(size_t tau_) const;

  void cleanup();
};

typedef AntichainNetwork<DynamicAlgoUDC> UDCNetwork;
typedef AntichainPtrs<DynamicAlgoUDC>::type UDCPtrs;
typedef acvertex_t<DynamicAlgoUDC>::type uvertex_t;
typedef acvertex_i<DynamicAlgoUDC>::type uvertex_i;
typedef ac_oute_i<DynamicAlgoUDC>::type u_oute_i;

}}

#endif
