#include "Utils.hpp"

namespace ig { namespace core { namespace util {

StateSharedPtr 
nextState(const Network& net_,
          const StateSharedPtr& statePtr_,
          const ActionSharedPtr& actionPtr_,
          const OrderedTaskSet& finished_,
          vertex_t u_)
{
  StateSharedPtr ret(new State(*statePtr_));
  ret->addInterdictedSet(*actionPtr_);
  ret->_active.erase(u_);
  ret->_interdicted.erase(u_);
  ret->_res -= actionPtr_->size();
  ret->_dormant.insert(u_);

  // These tasks have now finished
  OrderedTaskSet completed;
  std::set_union(finished_.begin(), finished_.end(),
                 ret->_dormant.begin(), ret->_dormant.end(),
                 std::inserter(completed, completed.begin()));
  completed.insert(u_);

  oute_i ei, ei_end;
  for(boost::tie(ei, ei_end) = boost::out_edges(u_, net_.graph()); ei != ei_end; ++ei)
  {
    vertex_t successor = boost::target(*ei, net_.graph());
    
    ine_i pei, pei_end;
    boost::tie(pei, pei_end) = boost::in_edges(u_, net_.graph());
    
    bool containsAll = true;
    for(;pei != pei_end; ++pei)
    {
      vertex_t pred = boost::target(*pei, net_.graph());
      if(completed.find(pred) == completed.end())
      {
        containsAll = false;
        break;
      }
    }
    if(containsAll)
    {
      ret->_active.insert(successor);
      ret->_dormant.erase(successor);
    }
  }
  return ret;
}

}}}
