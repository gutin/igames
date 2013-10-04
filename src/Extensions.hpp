#ifndef EXTENSIONS_HPP
#define EXTENSIONS_HPP

#include "DynamicAlgorithm.hpp"

namespace ig { namespace core {

struct ImplementationUncertaintyEvaluator
{
  static double evaluate(const UDC& udc, const UDCNetwork& unet_, uvertex_t uv_,
                  const Network& net_, 
                  const State& s, const ActionSharedPtr& candidate,
                  double totalRate, size_t budget_,
                  size_t fcode, StateTemplateMap& stmap);  
};

struct CrashingEvaluator
{
  static double evaluate(const UDC& udc, const UDCNetwork& unet_, uvertex_t uv_,
                  const Network& net_, 
                  const State& s, const ActionSharedPtr& candidate,
                  double totalRate, size_t budget_,
                  size_t fcode, StateTemplateMap& stmap);  
};

}}

#endif
