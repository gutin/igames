#include "Network.hpp"
#include "UDCNetwork.hpp"

#include "Task.hpp"
#include "StaticAlgorithms.hpp"
#include "Extensions.hpp"
#include "DynamicEvaluator.hpp"
#include "DynamicAlgorithm.hpp"

#include <iostream>
#include <sstream>

#include <boost/tokenizer.hpp>

using namespace ig::core;

int main(int ac_, char** av_)
{
  size_t budget;
  Network n;
  if(ac_ < 5)
  {
    std::cout << "<rcp-file> <budget> [determ|static] [impunc] [taskList]" << std::endl;
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
  bool crash = heurType == "crash";
  if(impunc) std::cout << "solving the implementation uncertianty version" << std::endl;
  StaticPolicy policy;
  double value = 0;
  n.import(rcpfile);
  if(determOrStatic == "determ")
  {
    if(ac_ > 5)
    {
      std::string taskListStr = av_[5];
      boost::char_separator<char> sep(",");
      boost::tokenizer<boost::char_separator<char> > tokens(taskListStr, sep);
      BOOST_FOREACH(const std::string& taskStr, tokens)
      {
        std::cout << "Will interdict " << taskStr << std::endl;
        policy << atoi(taskStr.c_str());
      }
    }
    else
    {
      deterministicPolicy(n, budget, policy, impunc);
    }
    if(impunc)
    {
      //USe an explicit Kulkarni solver to get the right value
      value = FastEvaluator<StaticPolicy, ImplementationUncertaintyEvaluator>(policy).evaluate(n, budget);
    }
    else if(crash)
    {
      //USe an explicit Kulkarni solver to get the right value
      value = FastEvaluator<StaticPolicy, CrashingEvaluator>(policy).evaluate(n, budget);
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
  else if(heurType == "static")
  {
    value = staticStochasticPolicy(n, budget, policy);
  }
  else 
  {
    if(impunc)
    {
      std::cout << "Solving the optimal impunc" << std::endl;
      value = DynamicAlgorithm<ImplementationUncertaintyEvaluator>().optimalValue(n, budget);
    }
    else if(crash)
    {
      std::cout << "Solving the optimal crashing" << std::endl;
      value = DynamicAlgorithm<CrashingEvaluator>().optimalValue(n, budget);
    }
    else
    {
      std::cout << "Solving the optimal standard" << std::endl;
      value = DynamicAlgorithm<StandardEvaluator>().optimalValue(n, budget);
    }
  }
  std::cout << value << std::endl;
  return 0;
}
