#include "Network.hpp"
#include "Simulation.hpp"

#include "Task.hpp"
#include "PersistedPolicy.hpp"
#include "DynamicEvaluator.hpp"
#include "StaticAlgorithms.hpp"

#include <boost/program_options.hpp>
#include <boost/program_options/value_semantic.hpp>

#include <iostream>
#include <sstream>

using namespace ig::core;
using namespace ig::experiment;

struct InterdictionSamplingStateVisitor
{
  OrderedTaskSet _interdictedTasks;
  std::vector<double> _realizedRates;

  void visit(const StateSharedPtr& /*state*/, const ActionSharedPtr& actionPtr_) 
  {
    BOOST_FOREACH(vertex_t t, *actionPtr_)
    {
      _interdictedTasks.insert(t);
    }
  }

  void recordCompletion(vertex_t t, double d)
  {
    _realizedRates[t] = 1.0/d;
  }

  InterdictionSamplingStateVisitor(const Network& net_)
    : _realizedRates(net_.size(), 0)
  {
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

template <class Policy, class Simulation>
void runSimulations(const Network& net_, size_t budget_, const Policy& policy_, Simulation& sim_, size_t nruns_, std::vector<size_t>& icounts, std::vector<size_t>& critcounts) 
{
  for(size_t i = 0; i < nruns_; ++i)
  {
    std::cout << "Running simulation " << i << std::endl;
    double makespan = sim_.run(budget_);
    BOOST_FOREACH(vertex_t t, sim_._visitor._interdictedTasks)
    {
      icounts[t]++;
    }

    //std::cout << "The realized durations are ";
    //std::copy(sim_._visitor._realizedRates.begin(), sim_._visitor._realizedRates.end(), std::ostream_iterator<double>(std::cout, " "));
    //std::cout << std::endl;
    // find critical tasks in that realization
    // first we use a hack to tmporarily change the task durations in the network by sawpping them with the vals recorded in the realization vector
    const_cast<Network&>(net_).swapDurations(sim_._visitor._realizedRates);

    StaticPolicies sps;
    CriticalPaths cps;
    ig::core::allOptimalDeterministicPolicies(net_, 0, sps, cps);
    //std::cout << cps.size() << " critical paths found in that realization " << std::endl;
    assert(cps.size());
    TaskList aCriticalPath = *cps.begin();
    //std::cout << "The crit path is ";
    //std::copy(aCriticalPath.begin(), aCriticalPath.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    //std::cout << std::endl;

    double len = 0;
    BOOST_FOREACH(vertex_t ct, aCriticalPath)
    {
      len += TASK_PROP(ct, expNormal);
      critcounts[ct]++;
    }
    assert(std::abs(len - makespan) < 1e-3);
    sim_._visitor._interdictedTasks.clear(); 

    //Crucially remember to change the durations back to their orginal numbers
    const_cast<Network&>(net_).swapDurations(sim_._visitor._realizedRates);
  }
}

template <class Policy>
void interdictionProbs(const Network& net_, size_t budget_, const Policy& policy_ , size_t nruns_, bool verbose_)
{
  std::cout << "Performing the experiment" << std::endl; 

  std::vector<size_t> icounts(net_.size(), 0);
  std::vector<size_t> critcounts(net_.size(), 0);
  InterdictionSamplingStateVisitor isv(net_);
  if(verbose_)
  {
    Simulation<Policy, InterdictionSamplingStateVisitor, false, true> sim(net_, policy_, isv);
    runSimulations(net_, budget_, policy_, sim, nruns_, icounts, critcounts);
  }
  else
  {
    Simulation<Policy, InterdictionSamplingStateVisitor> sim(net_, policy_, isv);
    runSimulations(net_, budget_, policy_, sim, nruns_, icounts, critcounts);
  }
  for(size_t i = 0; i < icounts.size(); ++i)
  {
    double prob = double(icounts[i]) / double(nruns_); 
    std::cout << "Probability of interdicting [" <<  i << "] = [" << prob << "]" << std::endl; 
  }
  std::cout << std::endl << "\nPrinting the criticality indices now\n" << std::endl;

  for(size_t i = 0; i < critcounts.size(); ++i)
  {
    double prob = double(critcounts[i]) / double(nruns_); 
    std::cout << "Criticality index [" <<  i << "] = [" << prob << "]" << std::endl; 
  }
}

void interdictionProbs(const Network& net_, size_t budget_, const std::string& pfile_, size_t nruns_, bool verbose_)
{
  PersistedPolicy ppol(pfile_.c_str(), net_);
  interdictionProbs(net_, budget_, ppol, nruns_, verbose_);
}

void interdictionProbs(const Network& net_, size_t budget_, size_t nruns_, bool verbose_)
{
  double optimalValue;
  DynamicPolicy optimalPolicy;
  DynamicAlgorithm<StandardEvaluator>().optimalPolicyAndValue(net_, budget_, optimalPolicy, optimalValue);
  interdictionProbs(net_, budget_, optimalPolicy, nruns_, verbose_);
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
    ("verbose,V","Should print detailed info?")
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
    interdictionProbs(n, budget, nruns, vm.count("verbose"));
    return 0;
  }

  std::string pfile = vm["policy-file"].as<std::string>();
  interdictionProbs(n, budget, pfile, nruns, vm.count("verbose"));
  return 0;
}
