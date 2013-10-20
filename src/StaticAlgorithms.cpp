#include "StaticAlgorithms.hpp"
#include "DynamicEvaluator.hpp"

#include <ilcplex/ilocplex.h>

#include <sstream>
#include <limits>

namespace
{

const IloNum M = 10000; 

using namespace ig::core;

struct VertexComp 
{
  const Network& net;

  VertexComp(const Network& net) : net(net) {}

  bool operator()(vertex_t one, vertex_t other)
  {
    return net.graph()[one].expNormal() > net.graph()[other].expNormal();  
  }
};

void populateStaticStochasticModel(const Network& net_, size_t budget_, IloEnv& env, IloModel& model, IloBoolVarArray theta)
{
  // Need information from antichains in the current implementation
  UDCNetwork unet(net_);
  
  // Get all states -- there could be thousands of these! 
  // This is ok because ultimately we're just preparing the model for it to be solved later
  StateCollection sc;
  DynamicAlgorithm<NullEvaluator>().execute(net_, 0, sc);
  
  std::cout << "Got the states .. all " << sc._statesAggregated.size() << " of them " << std::endl;  

  size_t index = 0;
  State startingState;
  util::startingState(net_, 0, startingState);

  // First add the objective, we will also generate the 'varphi' decision variables.. and
  // they will be indexed 0,..,|X| 
  IloNumVarArray varphi(env);
  BOOST_FOREACH(const State& state, sc._statesAggregated)
  {
    std::stringstream ss;
    ss << "varrphi_" << index;
    varphi.add(IloNumVar(env, (IloNum)0, std::numeric_limits<IloNum>::max(), ss.str().c_str()));
    if(state == startingState)
    {
      model.add(IloMaximize(env, varphi[index]));
    }
    ++index;
  }

  std::cout << "Added the objective function" << std::endl;

  index = 0; //reset index for iterating through tasks

  IloNumVarArray alpha(env), beta(env);
  
  typedef boost::unordered_map<State, size_t, StateHash> StateIndexMap;
  StateIndexMap stateIndexMap; 

  // Now we will the alpha and beta constraints
  vertex_i vi, vi_end;
  for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
  {
    std::cout << "alpha/beta contraints for vertex " << *vi << std::endl;
    // disregard start and end tasks
    if(net_.isStart(*vi) || net_.isEnd(*vi))
      continue;
    std::stringstream thetaLabel;
    thetaLabel << "theta_" << index;
    theta.add(IloBoolVar(env, thetaLabel.str().c_str()));
    
    size_t indexState = 0; //new index for states
    double gamma =net_.graph()[*vi]._nu - net_.graph()[*vi]._delta;
    BOOST_FOREACH(const State& state, sc._statesAggregated)
    {
      size_t matrixIndex = index * sc._statesAggregated.size() + indexState;
      std::stringstream alphaLabel, betaLabel;
      alphaLabel << "alpha_" << index << "_" << indexState;
      betaLabel << "beta_" << index << "_" << indexState;

      alpha.add(IloNumVar(env, IloNum(0), std::numeric_limits<IloNum>::max(), alphaLabel.str().c_str()));
      beta.add(IloNumVar(env, IloNum(0), std::numeric_limits<IloNum>::max(), betaLabel.str().c_str()));
      model.add(alpha[matrixIndex] <= M * theta[index]); 
      model.add(alpha[matrixIndex] <= gamma *varphi[indexState]); 
      model.add(beta[matrixIndex] >= gamma * varphi[indexState] - M * (1 - theta[index]));
      stateIndexMap[state] = indexState; 
      indexState++;
    }
    index++;
  }
  
  // The main constraints equating state values
  ActionSharedPtr empty(new Action);
  BOOST_FOREACH(const State& state, sc._statesAggregated)
  {
    index = stateIndexMap[state];
    if(util::isTerminalState(net_, state))
    {
      model.add(varphi[index] == 0);
      continue;
    }
    size_t activityIndex = 0;
    double sumRate = 0;
    BOOST_FOREACH(uvertex_t u, state._active)
    {
      sumRate += net_.graph()[u]._nu;
    }
    
    IloExpr lhsExpr(env), rhsExpr(env);
    lhsExpr += sumRate * varphi[index];
    BOOST_FOREACH(vertex_t u, state._active)
    {
      // Index of task 'u' in the decision vector is 'u-1' since we skipped put the start task
      lhsExpr -= alpha[(u-1) * sc._statesAggregated.size() + index];
    }

    BOOST_FOREACH(vertex_t u, state._active)
    {
      // Find which UDC this state belongs in
      OrderedTaskSet allTasksInState;
      std::set_union(state._active.begin(), state._active.end(), 
                     state._dormant.begin(), state._dormant.end(),
                     std::inserter(allTasksInState, allTasksInState.begin()));
      uvertex_i ui, ui_end;
      boost::tie(ui, ui_end) = boost::vertices(unet._ug);
      uvertex_i theUDC = ui_end;
      for(;ui != ui_end; ++ui)
      {
        if(unet[*ui]._taskSet == allTasksInState)
        {
          theUDC = ui;
          break;
        }
      }

      StateSharedPtr nextState = util::nextState(net_, state, empty, unet[*theUDC]._finished, u);   

      size_t nextStateIndex = stateIndexMap.at(*nextState);
      rhsExpr += net_.graph()[u]._nu * varphi[nextStateIndex]
                  - beta[(u-1) * sc._statesAggregated.size() + nextStateIndex];
    }
    model.add(lhsExpr == rhsExpr + 1);
  }

  // Some budget constraint
  IloExpr budgetExpr(env);
  size_t activityIndex = 0;
  for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
  {
    if(net_.isEnd(*vi) || net_.isStart(*vi))
      continue;
    budgetExpr += theta[activityIndex];
    activityIndex++;
  }
  model.add(budgetExpr == static_cast<IloInt>(budget_));
}

}

namespace ig { namespace core {

ActionSharedPtr StaticPolicy::at(const State& state_) const
{
  // In the static policy interdict whatever tasks are in the pattern and 
  // eligible for interdiction in the current state
  ActionSharedPtr ret(new Action(_targets));
  BOOST_FOREACH(vertex_t u, *ret)
  {
    // if the task is either not active or interdicted then we do not want
    //  it to be part of the action (even if it's in the pattern)
    if(state_._active.find(u) == state_._active.end() ||
       state_._interdicted.find(u) != state_._interdicted.end())
    {
      ret->erase(u);
    }
  }
  // This bit is only really necessary for things like implementation ucertainty
  // where the static strategy fails to see what is going on
  while(ret->size() > state_._res)
  {
    ret->erase(ret->begin());
  }
  return ret;
}

// adds a task to the interdiction pattern of this static policy.. just syntactic sugar
StaticPolicy& StaticPolicy::operator<<(vertex_t u)
{
  _targets.insert(u);
}

std::string StaticPolicy::asString() const
{
  std::stringstream ss;
  ss << "Theta=[ ";
  BOOST_FOREACH(vertex_t t, _targets)
  {
    ss << t << " ";
  }
  ss << "]";
  return ss.str();
}

// Work out a static policy from a determinsitic algorithm
double deterministicPolicy(const Network& net_, size_t budget_, StaticPolicy& policy_, bool impunc_)
{
  // Use a solver to get the optimal deterministic policy
  IloEnv env;
  IloModel model(env);

  // The main decision variables of the problem from which we derive the interdiction pattern
  IloBoolVarArray theta(env); 
  
  typedef std::map<vertex_t, size_t> VariableIndexMap; 
  std::map<vertex_t, VariableIndexMap> vars;
  VariableIndexMap endTasks;

  size_t u_index = 0;
  IloNumVarArray us(env), ds(env);  
  //Create the variables
  vertex_i vi, vi_end;
  for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
  {
    std::stringstream ss;
    ss << "x_" << *vi; 
    theta.add(IloBoolVar(env, ss.str().c_str()));

    ss.str("");
    ss << "d_" << *vi;
    ds.add(IloNumVar(env, IloNum(0), std::numeric_limits<IloNum>::max(), ss.str().c_str()));

    oute_i oei, oei_end;
    for(boost::tie(oei, oei_end) = boost::out_edges(*vi ,net_.graph()); oei != oei_end; ++oei)
    {
      vertex_t successor = boost::target(*oei, net_.graph());
      VariableIndexMap& sucs = vars[*vi];
      
      ss.str("");
      ss << "u_" << *vi << "_" << successor; 
      us.add(IloNumVar(env, IloNum(0), std::numeric_limits<IloNum>::max(), ss.str().c_str()));
      sucs[successor] = u_index;
      u_index++;

      oute_i oesi, oesi_end;
      boost::tie(oesi, oesi_end)  = boost::out_edges(successor, net_.graph());
      if(oesi == oesi_end)
      {
        ss.str("");
        ss << "u_" << successor << "_t";
        us.add(IloNumVar(env, IloNum(0), std::numeric_limits<IloNum>::max(), ss.str().c_str()));
        endTasks[successor] = u_index;
        u_index++;
      }
    }
  }

  //Create the objective function
  IloExpr objectiveExpr(env);
  BOOST_FOREACH(VariableIndexMap::value_type& entry, endTasks)
  {
    objectiveExpr += net_.graph()[entry.first].expNormal() * us[entry.second];
  }
  for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
  {
    oute_i oei, oei_end;
    for(boost::tie(oei, oei_end) = boost::out_edges(*vi ,net_.graph()); oei != oei_end; ++oei)
    {
      vertex_t successor = boost::target(*oei, net_.graph());
      objectiveExpr += net_.graph()[*vi].expNormal() * us[vars.at(*vi).at(successor)];
    }
    objectiveExpr += ds[*vi];
  }
  model.add(IloMaximize(env, objectiveExpr));

  // Constraints
  for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
  {
    // Flow constraints first
    IloExpr flowConExpr(env);
    ine_i iei, iei_end;
    boost::tie(iei, iei_end) = boost::in_edges(*vi, net_.graph());
    for(;iei != iei_end; ++iei)
    {
      vertex_t pred = boost::source(*iei, net_.graph());
      flowConExpr += us[vars.at(pred).at(*vi)];

    }
    oute_i oesi, oesi_end;
    boost::tie(oesi, oesi_end)  = boost::out_edges(*vi, net_.graph());
    if(oesi == oesi_end)
    {
      flowConExpr -= us[endTasks.at(*vi)];
    }
    for(;oesi != oesi_end; ++oesi)
    {
      vertex_t succ = boost::target(*oesi, net_.graph());
      flowConExpr -= us[vars.at(*vi).at(succ)];
    }
    model.add(flowConExpr <= 0);

    // Then control the delay variable through big-M tricks
    IloExpr bound(env), bound2(env);
    bound += M * theta[*vi];
    double advantage = net_.graph()[*vi].expDelayed() - net_.graph()[*vi].expNormal();
    if(impunc_)
    {
      // S
      advantage *= TASK_ATTR(*vi, _probDelaySuccess);
    }
    if(advantage <= 0 && !net_.isEnd(*vi) && !net_.isStart(*vi))
      std::cout << "Warning.. no advantage to interdicting " << *vi << std::endl;

    boost::tie(oesi, oesi_end)  = boost::out_edges(*vi, net_.graph());
    if(oesi == oesi_end)
    {
      bound2 += advantage * us[endTasks.at(*vi)];
    }
    for(;oesi != oesi_end; ++oesi)
    {
      vertex_t succ = boost::target(*oesi, net_.graph());
      bound2 += advantage * us[vars.at(*vi).at(succ)];
    }

    model.add(ds[*vi] <= bound);
    model.add(ds[*vi] <= bound2);
  }
  IloExpr finalFlowConExpr(env);
  BOOST_FOREACH(VariableIndexMap::value_type& entry, endTasks)
  {
    finalFlowConExpr += us[entry.second];
  }
  model.add(finalFlowConExpr <= 1);

  // Now the budget constraint
  IloExpr budgetExpr(env);
  size_t activityIndex = 0;
  for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
  {
    budgetExpr += theta[*vi];
  }
  model.add(budgetExpr == static_cast<IloInt>(budget_));
  std::cout << "Finished building deterministic model. Solving." << std::endl;
  try 
  {
    IloCplex cplex(model);
    // Uncomment below line to debug
    cplex.exportModel("determ.lp");
    cplex.solve();

    env.out() << "Solution status = " << cplex.getStatus() << std::endl;
    env.out() << "Solution value  = " << cplex.getObjValue() << std::endl;

    for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
    {
      if(std::abs(cplex.getValue(theta[*vi]) - 1) < 1e-02)
        policy_ << *vi;
    }
    std::cout << "The static policy is " << policy_.asString() << std::endl;
    return cplex.getObjValue();
  }
  catch(IloException& e)
  {
    std::cerr << "Ilo exception caught: " << e << std::endl;
  }
  catch(...)
  {
    std::cerr << "Some other unknown exception" << std::endl;
  }
  return 0;
}

double staticStochasticPolicy(const Network& net_, size_t budget_, StaticPolicy& policy_)
{
  // Use a solver to get the optimal static stochastic policy
  IloEnv env;
  IloModel model(env);

  // The main decision variables of the problem from which we derive the interdiction pattern
  IloBoolVarArray theta(env); 
  populateStaticStochasticModel(net_, budget_, env, model, theta); 

  try 
  {
    IloCplex cplex(model);
    //NOTE: Uncomment this line to debug the problem
    //cplex.exportModel("test.lp");
    std::cout << "Finished constructing the CPLEX model for solving static stochastic problem" << std::endl;
    cplex.solve();

    env.out() << "Solution status = " << cplex.getStatus() << std::endl;
    env.out() << "Solution value  = " << cplex.getObjValue() << std::endl;

    vertex_i vi, vi_end;
    for(boost::tie(vi, vi_end) = boost::vertices(net_.graph()); vi != vi_end; ++vi)
    {
      if(net_.isStart(*vi) || net_.isEnd(*vi))
        continue;
      if(cplex.getValue(theta[(*vi-1)]) == 1)
        policy_ << *vi;
    }
    std::cout << "The static policy is " << policy_.asString() << std::endl;
    return cplex.getObjValue();
  }
  catch(IloException& e)
  {
    std::cerr << "Ilo exception caught: " << e << std::endl;
  }
  catch(...)
  {
    std::cerr << "Some other unknown exception" << std::endl;
  }
  return 0;
}

void dumpStaticStochasticMILP(const Network& net_, size_t budget_, const std::string& lpFile_)
{
  IloEnv env;
  IloModel model(env);

  IloBoolVarArray theta(env); 
  populateStaticStochasticModel(net_, budget_, env, model, theta); 

  IloCplex cplex(model);
  cplex.exportModel(lpFile_.c_str());
}

double staticStochasticPolicyHeuristic(const Network& net_, size_t budget_, StaticPolicy& policy_)
{
  TaskList nLongestRunning;
  vertex_i vi, vi_end;
  boost::tie(vi, vi_end) = boost::vertices(net_.graph());
  std::copy(vi, vi_end, std::inserter(nLongestRunning, nLongestRunning.begin()));
  std::sort(nLongestRunning.begin(), nLongestRunning.end(), VertexComp(net_));
  nLongestRunning.resize(budget_ + 1);
  return staticStochasticPolicyHeuristic(net_, budget_, nLongestRunning, policy_);
}

double staticStochasticPolicyHeuristic(const Network& net_, size_t budget_, const TaskList& searchSpace_, StaticPolicy& policy_)
{
  double optimalHeuristicValue = std::numeric_limits<double>::min();
  OrderedTaskSetPtr bestChoice;
  for(size_t bits = 0; bits < (1L << searchSpace_.size()); ++bits)
  {
    if(util::numBitsSet(bits) != budget_)
      continue;
    OrderedTaskSetPtr chosen(new OrderedTaskSet);
    for(size_t b = 0; b < searchSpace_.size(); ++b)
    {
      if((1L << b) & bits)
      {
        vertex_t t = searchSpace_[b];
        chosen->insert(searchSpace_[b]);
        
        // A cute little hack where we swap the nu and delta temporarily and eval with 0 budget..
        std::swap(const_cast<double&>(net_.graph()[t]._delta), const_cast<double&>(net_.graph()[t]._nu));
      }
    }
    double value = DynamicAlgorithm<StandardEvaluator>().optimalValue(net_, 0);
    if(value > optimalHeuristicValue)
    {
      std::cout << "Improving to " << value << std::endl;
      std::cout << "with: ";
      BOOST_FOREACH(vertex_t c, *chosen)
      {
        std::cout << c << " ";
      }
      std::cout << std::endl;
      optimalHeuristicValue = value;
      bestChoice = chosen;
    }
    // undo the effects of the hack
    BOOST_FOREACH(vertex_t t, *chosen)
    {
      std::swap(const_cast<double&>(net_.graph()[t]._delta), const_cast<double&>(net_.graph()[t]._nu));
    }
  }
  BOOST_FOREACH(vertex_t t, *bestChoice)
  {
    policy_ << t;
  }
  return optimalHeuristicValue;
}

}}
