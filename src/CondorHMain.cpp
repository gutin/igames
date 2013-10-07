#include "Network.hpp"
#include "UDCNetwork.hpp"

#include "Task.hpp"
#include "StaticAlgorithms.hpp"
#include "Extensions.hpp"
#include "DynamicEvaluator.hpp"

#include <iostream>
#include <sstream>

using namespace ig::core;

int main(int ac_, char** av_)
{
  size_t budget;
  Network n;
  if(ac_ != 5)
  {
    std::cout << "<rcp-file> <budget> [determ|static] [impunc]" << std::endl;
    return 1;
  }
  
  std::string rcpfile = av_[1];
  std::cout << "RCP file is " << rcpfile << std::endl;
  budget = atoi(av_[2]);
  std::cout << "Budget is " << budget << std::endl;
  std::string determOrStatic = av_[3];
  std::cout << "Determ or stati is " << determOrStatic << std::endl;
  std::string heurType = av_[4];
  std::cout << "heur type is " << heurType << std::endl;
  bool impunc = heurType == "impunc";
  if(impunc) std::cout << "solving the implementation uncertianty version" << std::endl;
  StaticPolicy policy;
  double value = 0;
  n.import(rcpfile);
  if(determOrStatic == "determ")
  {
    deterministicPolicy(n, budget, policy, impunc);
    if(impunc)
    {
      //USe an explicit Kulkarni solver to get the right value
      value = FastEvaluator<StaticPolicy, ImplementationUncertaintyEvaluator>(policy).evaluate(n, budget);
    }
    else
    {
      // Now set any tasks in the interdiction pattern to their delayed rate and then 'solve'
      // the efficient dynamic algorithm with 0 budget
      vertex_i vi, vi_end;
      for(boost::tie(vi, vi_end) = boost::vertices(n.graph()); vi != vi_end; ++vi)
      {
        if(policy._targets.find(*vi) != policy._targets.end())
        {
          const_cast<Task&>(n.graph()[*vi])._nu = n.graph()[*vi]._delta;
        }
      }
      value = DynamicAlgorithm<StandardEvaluator>().optimalValue(n, 0);
    }
  }
  else 
  {
    value = staticStochasticPolicy(n, budget, policy);
  }
  std::cout << value << std::endl;
  return 0;
}
