#include "DynamicAlgoUDC.hpp"
#include "Utils.hpp"

namespace ig { namespace core {

void DynamicAlgoUDC::store(size_t tau_, double val_)
{
  // Can we optimise by pre-sizing the vectors?
  _v.push_back(val_);
  _indexes.push_back(tau_);
}

double DynamicAlgoUDC::value(size_t tav_) const
{
  std::vector<size_t>::const_iterator indexIter = util::binary_search(_indexes.begin(),
                                                                    _indexes.end(),
                                                                    tav_);
  if(indexIter == _indexes.end())
  {
    std::cout << "Error! Failed to value for " << tav_ << std::endl;
    abort();
  }
  return _v[std::distance(_indexes.begin(), indexIter)];
}

void DynamicAlgoUDC::cleanup()
{
  _indexes.clear();
  _v.clear();
  UDC::cleanup();
}

}}
