#include "Network.hpp"
#include "UDCNetwork.hpp"

#include "Task.hpp"
#include "DynamicAlgorithm.hpp"

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

  const char* DESCRIPTION = "Interidction game solver";
}

int main(int ac_, char** av_)
{
  size_t size, budget, instance;
  double os;
  std::string baseDir;

  namespace po = boost::program_options;
  po::options_description desc(DESCRIPTION);
  desc.add_options() 
    ("help", "Print help messages") 
    (SIZE_ARG_NAME, po::value<size_t>(&size), "Size of the network") 
    (BUDGET_ARG_NAME, po::value<size_t>(&budget), "Interdiction budget")
    (INSTANCE_ARG_NAME, po::value<size_t>(&instance), "Instance number")
    (ORDER_STRENGTH_ARG_NAME, po::value<double>(&os), "Order strength")
    (RCPBASE_ARG_NAME, po::value<std::string>(&baseDir), "Base directory with .rcp files");

  po::variables_map vm;
  po::store(po::parse_command_line(ac_, av_, desc), vm);
  po::notify(vm);

  if(!vm.count(RCPBASE_ARG_NAME) ||
     !vm.count(BUDGET_ARG_NAME) ||
     !vm.count(INSTANCE_ARG_NAME) ||
     !vm.count(ORDER_STRENGTH_ARG_NAME) ||
     !vm.count(SIZE_ARG_NAME) 
     )
  {
    std::cout << desc << std::endl;
    return 1;
  }

  std::stringstream filesstr;
  filesstr << baseDir;
  filesstr << "/" << size << "-OS-" << os;
  filesstr << "/Pat" << instance << ".rcp";

  Network n;
  n.import(filesstr.str());

  double value = DynamicAlgorithm<StandardEvaluator>::optimalValue(n, budget);
  std::cout << "Value is " << value << std::endl;
  return 0;
}
