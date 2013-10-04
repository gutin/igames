#include "Network.hpp"
#include "Simulation.hpp"

#include "Task.hpp"
#include "PersistedPolicy.hpp"
#include "DynamicEvaluator.hpp"

#include <boost/program_options.hpp>
#include <boost/program_options/value_semantic.hpp>

#include <iostream>
#include <sstream>

using namespace ig::core;
using namespace ig::experiment;

struct InterdictionSamplingStateVisitor
{
  OrderedTaskSet _interdictedTasks;

  void visit(const StateSharedPtr& /*state*/, const ActionSharedPtr& actionPtr_) 
  {
    BOOST_FOREACH(vertex_t t, *actionPtr_)
    {
      _interdictedTasks.insert(t);
    }
  }
};

namespace 
{
  const char* SIZE_ARG_NAME = "size";
  const char* BUDGET_ARG_NAME = "budget";
  const char* INSTANCE_ARG_NAME = "instance";
  const char* ORDER_STRENGTH_ARG_NAME = "os";
  const char* RCPBASE_ARG_NAME = "rcp-base";
  const char* RCPFILE_ARG_NAME = "rcp-file";

  const char* DESCRIPTION = "Interdiction game solver";
}

void interdictionProbs(const Network& net_, size_t budget_, const std::string& pfile_, size_t nruns_)
{
  std::cout << "Performing the experiment" << std::endl; 
  PersistedPolicy ppol(pfile_, net_);

  std::vector<size_t> icounts(net_.size(), 0);
  InterdictionSamplingStateVisitor isv;
  Simulation<PersistedPolicy, InterdictionSamplingStateVisitor> sim(net_, ppol, isv);
  for(size_t i = 0; i < nruns_; ++i)
  {
    std::cout << "Running simulation " << i << std::endl;
    sim.run(budget_);
    BOOST_FOREACH(vertex_t t, isv._interdictedTasks)
    {
      icounts[t]++;
    }
    isv._interdictedTasks.clear(); 
  }

  for(size_t i = 0; i < icounts.size(); ++i)
  {
    double prob = double(icounts[i]) / double(nruns_); 
    std::cout << "Probability of interdicting [" <<  i << "] = [" << prob << "]" << std::endl; 
  }
}

int main(int ac_, char** av_)
{
  size_t budget, nruns = 1000;

  namespace po = boost::program_options;
  po::options_description desc(DESCRIPTION);
  desc.add_options() 
    ("help", "Print help messages") 
    (SIZE_ARG_NAME, po::value<size_t>(), "Size of the network") 
    (BUDGET_ARG_NAME, po::value<size_t>(&budget), "Interdiction budget")
    ("runs", po::value<size_t>(&nruns), "Number of runs")
    (INSTANCE_ARG_NAME, po::value<size_t>(), "Instance number")
    (ORDER_STRENGTH_ARG_NAME, po::value<double>(), "Order strength")
    (RCPBASE_ARG_NAME, po::value<std::string>(), "Base directory with .rcp files")
    ("delays-from-file,D", "Should delayed durations be taken from the .rcp file?")
    ("policy-file,P",po::value<std::string>(), "Persistence file to read from")
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

  
  double value = 0;
  if(!vm.count("policy-file"))
  {
    std::cout << "No policy file given. Try --help" << std::endl;
    return 1;
  }

  std::string pfile = vm["policy-file"].as<std::string>();
  interdictionProbs(n, budget, pfile, nruns);
  return 0;
}
