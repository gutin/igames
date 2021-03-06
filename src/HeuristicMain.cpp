#include "Network.hpp"
#include "UDCNetwork.hpp"

#include "Task.hpp"
#include "StaticAlgorithms.hpp"
#include "Extensions.hpp"
#include "DynamicEvaluator.hpp"
#include <boost/program_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/tokenizer.hpp>

#include <iostream>
#include <iterator>
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
  const char* STATIC_HEURISTIC_OPTION = "heurstatic";
  const char* MINIMAL_STATIC_HEURISTIC_OPTION = "minhstatic";
  const char* DUMP_STATIC_OPTION = "dump-static";
  const char* DETERMINISTIC_OPTION = "determ";
  const char* SELECTEVAL_OPTION = "select";
  const char* DUMP_PROJECT_GRAPH_OPTION = "dumpdot";

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
    (STATIC_HEURISTIC_OPTION, "Use the heuristic stochastic static solution?")
    (DUMP_STATIC_OPTION, po::value<std::string>(), "Dump the stochastic static problem as the given .lp file")
    (DUMP_PROJECT_GRAPH_OPTION, po::value<std::string>(), "Dump the project as a DOT file")
    ("impunc", "Solve with implementation uncertainty?")
    ("crash", "Solve with crashing?")
    (DETERMINISTIC_OPTION, "Use the deterministic solution?")
    (RCPFILE_ARG_NAME, po::value<std::string>(), "Direct .rcp file to use")
    ("pcps", "Print deterministic critical paths")
    (SELECTEVAL_OPTION, po::value<std::string>(), "Evaluate with interdicited tasks (a comma separated list)");

  po::variables_map vm;
  po::store(po::parse_command_line(ac_, av_, desc), vm);
  po::notify(vm);

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

  if(vm.count(DUMP_PROJECT_GRAPH_OPTION))
  {
    std::string dotFile = vm[DUMP_PROJECT_GRAPH_OPTION].as<std::string>();
    n.exportDot(dotFile);
    return 0;
  }
  
  if(!vm.count(BUDGET_ARG_NAME))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  if(vm.count("pcps"))
  {
    StaticPolicies sps;
    CriticalPaths cps;
    std::cout << " value is " << allOptimalDeterministicPolicies(n, budget, sps, cps) << std::endl;
    BOOST_FOREACH(StaticPolicy& sp, sps)
    {
      std::cout << "One choice: ";
      std::copy(sp._targets.begin(), sp._targets.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
      std::cout << std::endl;
    }
    return 0;
  }

  bool impunc = vm.count("impunc");
  //Work out which algorithm to run...
  double value = 0; 
  StaticPolicy policy;

  if(vm.count(DETERMINISTIC_OPTION))
  {
    if(vm.count(SELECTEVAL_OPTION))
    {
      std::string taskListStr = vm[SELECTEVAL_OPTION].as<std::string>();
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
    else
    {
      if(vm.count("crash"))
      {
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
  }
  else if(vm.count(STATIC_HEURISTIC_OPTION))
  {
    if(vm.count("crash"))
      value = staticStochasticPolicyHeuristic<CrashingEvaluator>(n, budget, policy);
    else if(vm.count("impunc"))
      value = staticStochasticPolicyHeuristic<ImplementationUncertaintyEvaluator>(n, budget, policy);
    else
      value = staticStochasticPolicyHeuristic<StandardEvaluator>(n, budget, policy);
  }
  else if(vm.count(MINIMAL_STATIC_HEURISTIC_OPTION))
  {
    if(vm.count("crash"))
      value = minimalStaticStochasticPolicyHeuristic<CrashingEvaluator>(n, budget, policy);
    else if(vm.count("impunc"))
      value = minimalStaticStochasticPolicyHeuristic<ImplementationUncertaintyEvaluator>(n, budget, policy);
    else
      value = minimalStaticStochasticPolicyHeuristic<StandardEvaluator>(n, budget, policy);
  }
  else if(vm.count(STATIC_OPTION))
  {
    value = staticStochasticPolicy(n, budget, policy);
  }
  else if(vm.count(DUMP_STATIC_OPTION))
  {
    dumpStaticStochasticMILP(n, budget, vm[DUMP_STATIC_OPTION].as<std::string>());
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
