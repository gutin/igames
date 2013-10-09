#include "Network.hpp"
#include "Simulation.hpp"

#include "Task.hpp"
#include "PersistedPolicy.hpp"
#include "DynamicEvaluator.hpp"


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
  if(ac_ == 1)
  {
    std::cout << "<rcp-file> <policy-file> <budget> [<runs>]" << std::endl;
    return 1;
  }
  Network n;
  std::string rcpFile = av_[1];
  n.import(rcpFile, false);
  std::cout << "There are " << boost::num_edges(n.graph()) << " edges" << std::endl;
  std::cout << "There are " << boost::num_vertices(n.graph()) << " vertices" << std::endl;

  std::string pfile = av_[2];
  budget = atoi(av_[3]);
  if(ac_ > 4)
  {
    nruns = atoi(av_[4]);
  }

  interdictionProbs(n, budget, pfile, nruns);
  return 0;
}
