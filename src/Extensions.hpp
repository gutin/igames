#ifndef EXTENSIONS_HPP
#define EXTENSIONS_HPP

#include "DynamicAlgorithm.hpp"

namespace ig { namespace core {

struct ImplementationUncertaintyEvaluator
{
  double evaluate(const UDC& udc, const UDCNetwork& unet_, uvertex_t uv_,
                  const Network& net_, 
                  const State& s, const Action& candidate,
                  double totalRate, size_t budget_,
                  size_t fcode, StateTemplateMap& stmap) const;  

  StandardEvaluator _stdEval;
};

struct CrashingEvaluator
{
  double evaluate(const UDC& udc, const UDCNetwork& unet_, uvertex_t uv_,
                  const Network& net_, 
                  const State& s, const Action& candidate,
                  double totalRate, size_t budget_,
                  size_t fcode, StateTemplateMap& stmap) const;  
};

}}

#endif
