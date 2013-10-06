#include "Network.hpp"
#include "UDCNetwork.hpp"

#include "Task.hpp"
#include "StaticAlgorithms.hpp"
#include "Extensions.hpp"
#include "DynamicEvaluator.hpp"
#include <boost/program_options.hpp>
#include <boost/program_options/value_semantic.hpp>

#include <iostream>
#include <sstream>

using namespace ig::core;

namespace 
{
  const char* SIZE_ARG_NAME = "size";
  const char* BUDGET_ARG_NAME = "budget";
  const char* INSTANCE_ARG_NAME = "instance";
  const char* ORDER_STRENGTH_ARG_NAME = "os";
  const char* RCPBASE_ARG_NAME = "rcp-base";
  const char* RCPFILE_ARG_NAME = "rcp-file";
  const char* STATIC_OPTION = "static";
  const char* DETERMINISTIC_OPTION = "determ";

  const char* DESCRIPTION = "Heuristic interdiction game solver";
}

int main(int ac_, char** av_)
{
  size_t budget;

  namespace po = boost::program_options;
  po::options_description desc(DESCRIPTION);
  desc.add_options() 
    ("help", "Print help messages") 
    (SIZE_ARG_NAME, po::value<size_t>(), "Size of the network") 
    (BUDGET_ARG_NAME, po::value<size_t>(&budget), "Interdiction budget")
    (INSTANCE_ARG_NAME, po::value<size_t>(), "Instance number")
    (ORDER_STRENGTH_ARG_NAME, po::value<double>(), "Order strength")
    (RCPBASE_ARG_NAME, po::value<std::string>(), "Base directory with .rcp files")
    ("delays-from-file,D", "Should delayed durations be taken from the .rcp file?")
    (STATIC_OPTION, "Use the stochastic static solution?")
    ("impunc", "Solve with implementation uncertainty?")
    (DETERMINISTIC_OPTION, "Use the deterministic solution?")
    (RCPFILE_ARG_NAME, po::value<std::string>(), "Direct .rcp file to use");

  po::variables_map vm;
  po::store(po::parse_command_line(ac_, av_, desc), vm);
  po::notify(vm);

  if(!vm.count(BUDGET_ARG_NAME))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  bool delayFromFile = vm.count("delays-from-file") > 0;
  if(delayFromFile)
  {
    std::cout << "Using delayed durations from the file" << std::endl;
  }

  Network n;
  std::string rcpFile;

  if(vm.count(SIZE_ARG_NAME) && vm.count(INSTANCE_ARG_NAME) &&
     vm.count(ORDER_STRENGTH_ARG_NAME) && vm.count(RCPBASE_ARG_NAME))
  {
    size_t size = vm[SIZE_ARG_NAME].as<size_t>(); 
    size_t instance = vm[INSTANCE_ARG_NAME].as<size_t>();
    double os = vm[ORDER_STRENGTH_ARG_NAME].as<double>();

    std::string baseDir = vm[RCPBASE_ARG_NAME].as<std::string>();

    std::stringstream filesstr;
    filesstr << baseDir;
    filesstr << "/" << size << "-OS-" << os;
    filesstr << "/Pat" << instance << ".rcp";
    rcpFile = filesstr.str();
  }
  else if(vm.count(RCPFILE_ARG_NAME))
  {
    rcpFile = vm[RCPFILE_ARG_NAME].as<std::string>(); 
  }
  else
  {
    std::cout << "Must provide either:" << std::endl;
    std::cout << "\ta) Network size, instance number, budget and a base directory to look up the rcp file" << std::endl;
    std::cout << "\tb) The full path to a .rcp file" << std::endl;
    std::cout << "The help message below has more detail:\n\n" << std::endl;

    std::cout << desc << std::endl;
    return 1;
  }
  n.import(rcpFile, delayFromFile);
  std::cout << "There are " << boost::num_edges(n.graph()) << " edges" << std::endl;
  std::cout << "There are " << boost::num_vertices(n.graph()) << " vertices" << std::endl;
  
  bool impunc = vm.count("impunc");
  //Work out which algorithm to run...
  double value = 0; 
  StaticPolicy policy;
  if(vm.count(DETERMINISTIC_OPTION))
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
  else if(vm.count(STATIC_OPTION))
  {
    value = staticStochasticPolicy(n, budget, policy);
  }
  else
  {
    std::cerr << "Must specify either --static or --determ" << std::endl;
    std::cerr << ":" << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }
  std::cout << value << std::endl;
  return 0;
}
