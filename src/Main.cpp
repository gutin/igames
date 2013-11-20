#include "Network.hpp"
#include "UDCNetwork.hpp"

#include "Task.hpp"
#include "DynamicAlgorithm.hpp"
#include "Extensions.hpp"

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
  const char* COMPLEXITY_ARG_NAME = "complexity";

  const char* DESCRIPTION = "Interdiction game solver";
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
    (RCPBASE_ARG_NAME, po::value<std::string>(), "RCP file base directory")
    ("impunc", "Version with implementation uncertainty?")
    ("crash", "Version with crashing?")
    ("delays-from-file,D", "Should delayed durations be taken from the .rcp file?")
    (COMPLEXITY_ARG_NAME, "Print complexity?")
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
  std::cout << "Solving for budget of " << budget << std::endl;

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

  if(vm.count(COMPLEXITY_ARG_NAME))
  {
    UDCNetwork unet(n);
    std::map<size_t,size_t> chm;
    std::cout << "Complexity is [" << unet.complexity(budget, chm) << "]" << std::endl;
    BOOST_FOREACH(UDCNetwork::ComplexityHeatMap::value_type& kv, chm)
    {
      std::cout << "For complexity " << kv.first << " = " << kv.second << std::endl;
    }
    return 0;
  }
  
  double value = 0;
  if(vm.count("crash"))
  {
    value = DynamicAlgorithm<CrashingEvaluator>().optimalValue(n, budget);
  }
  else if(vm.count("impunc"))
  {
    value = DynamicAlgorithm<ImplementationUncertaintyEvaluator>().optimalValue(n, budget);
  }
  else
  {
    value = DynamicAlgorithm<StandardEvaluator>().optimalValue(n, budget);
  }
  std::cout << value << std::endl;
  return 0;
}
