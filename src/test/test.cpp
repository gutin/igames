#define BOOST_TEST_MODULE dynamicAlgoTests
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <Network.hpp>
#include <UDCNetwork.hpp>
#include <PersistedPolicy.hpp>
#include <DynamicAlgorithm.hpp>
#include <DynamicEvaluator.hpp>
#include <StaticAlgorithms.hpp>
#include <Simulation.hpp>
#include <Extensions.hpp>

using namespace ig::core;
using namespace ig::experiment;

// Testing the dynamic stochastic algorithm
BOOST_AUTO_TEST_SUITE( standardAlgorithm )

  // Make sure the optimal value and the value of 
  // the optimal policy worked out are equal
  BOOST_AUTO_TEST_CASE( kulkarniResultAndOptimalValueMustMatch )
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    double optValue = 0;
    DynamicPolicy policy;
    DynamicAlgorithm<StandardEvaluator>().optimalPolicyAndValue(n, B, policy, optValue);

    double workedOutVal = StandardDynamicEvaluator().evaluate(n, B, policy);

    BOOST_CHECK_CLOSE(optValue, workedOutVal, 1e-05);
  }

  // Make sure the optimal value and the value of 
  // the optimal policy worked out are equal
  BOOST_AUTO_TEST_CASE( kulkarniResultAndOptimalValueMustMatchForImplUnc )
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    double optValue = 0;
    DynamicPolicy policy;
    DynamicAlgorithm<ImplementationUncertaintyEvaluator>().optimalPolicyAndValue(n, B, policy, optValue);

    double workedOutVal = ImplUncertaintyEvaluator().evaluate(n, B, policy);

    BOOST_CHECK_CLOSE(optValue, workedOutVal, 1e-05);

    // check an exact result
    vertex_i vi, vi_end;
    for(boost::tie(vi, vi_end) = boost::vertices(n.graph()); vi != vi_end; ++vi)
    {
      const_cast<Task&>(n.graph()[*vi])._probDelaySuccess = 0.2;
    }
    policy.clear();
    DynamicAlgorithm<ImplementationUncertaintyEvaluator>().optimalPolicyAndValue(n, B, policy, optValue);
    BOOST_CHECK_CLOSE(40.3619, optValue, 1e-04);
    BOOST_CHECK_CLOSE(40.3619, ImplUncertaintyEvaluator().evaluate(n,B,policy), 1e-04);
  }

  BOOST_AUTO_TEST_CASE( dataQuality )
  {
    Network n;
    n.import("../samples/10-OS-0.8/Pat11.rcp");
    BOOST_CHECK_CLOSE(0.8, n.orderStrength(), 1e-06);
  }
BOOST_AUTO_TEST_SUITE_END()
  
// Testing the static stochastic algorthm
BOOST_AUTO_TEST_SUITE( staticStochasticAlgorithm )

  // Make sure the optimal value and the value of 
  // the optimal policy worked out are equal
  BOOST_AUTO_TEST_CASE( objectValueOfMILPMustMatchEvaluatedOne )
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    StaticPolicy staticPolicy;
    double expectedValue = staticStochasticPolicy(n, B, staticPolicy);

    BOOST_CHECK_CLOSE(expectedValue, StandardDynamicEvaluator().evaluate(n, B, staticPolicy), 1e-05);
  }

  // Test the deterministic algorithm
  BOOST_AUTO_TEST_CASE( deterministicMILP )
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    StaticPolicy staticPolicy;
    double actVal = deterministicPolicy(n, B, staticPolicy);
    BOOST_REQUIRE_EQUAL(54, actVal);
    BOOST_REQUIRE_EQUAL(3, staticPolicy._targets.size());
  }

  // Test the deterministic algorithm for impunc
  BOOST_AUTO_TEST_CASE( deterministicMILPForImpunc )
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    StaticPolicy staticPolicy;
    double actVal = deterministicPolicy(n, B, staticPolicy, true);
    BOOST_REQUIRE_EQUAL(B, staticPolicy._targets.size());

    std::cout << "Working out the exact mean of the Impunc static policy" << std::endl;
    double simulatedVal = ImplUncertaintyEvaluator().evaluate(n, B, staticPolicy);

    NullStateVisitor vnull;
    Simulation<StaticPolicy, NullStateVisitor, true> sim(n, staticPolicy, vnull);
    double mean = 0;
    size_t N = 10000L;
    for(size_t i = 0; i < N; ++i)
    {
      mean += sim.run(B);
    }
    mean /= N*1.0;

    BOOST_CHECK_CLOSE(simulatedVal, mean, 1);
  }

  BOOST_AUTO_TEST_CASE( testFastEvaluatorForImpunc)
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    StaticPolicy staticPolicy;
    double actVal = deterministicPolicy(n, B, staticPolicy, true);
    BOOST_REQUIRE_EQUAL(B, staticPolicy._targets.size());

    std::cout << "Working out the exact mean of the Impunc static policy" << std::endl;
    double meanVal = ImplUncertaintyEvaluator().evaluate(n, B, staticPolicy);

    BOOST_REQUIRE_EQUAL(B, staticPolicy._targets.size());
    double fastMeanVal = FastEvaluator<StaticPolicy, ImplementationUncertaintyEvaluator>(staticPolicy).evaluate(n, B);
    BOOST_CHECK_CLOSE(meanVal, fastMeanVal, 1e-4);
  }

BOOST_AUTO_TEST_SUITE_END()

// Testing the persisted policy
BOOST_AUTO_TEST_SUITE( persistedPolicy )

  // Make sure the optimal value and the value of 
  // the optimal policy worked out are equal
  BOOST_AUTO_TEST_CASE( objectValueMustMatchEvaluated )
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    double optValue = 0;
    {
      PersistantStoragePolicy psp("bla.policy", n);
      optValue = DynamicAlgorithm<StandardEvaluator>().execute(n, B, psp);
    }
    PersistedPolicy ppol("bla.policy", n);
    double workedOutVal = StandardDynamicEvaluator().evaluate(n, B, ppol);

    BOOST_CHECK_CLOSE(optValue, workedOutVal, 1e-05);
  }

  BOOST_AUTO_TEST_CASE(testSimulationAccurary)
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");

    double optValue = 0;
    DynamicPolicy policy;
    DynamicAlgorithm<StandardEvaluator>().optimalPolicyAndValue(n, B, policy, optValue);
    
    NullStateVisitor vnull;
    Simulation<DynamicPolicy, NullStateVisitor> sim(n, policy, vnull);
    double mean = 0;
    size_t N = 10000L;
    for(size_t i = 0; i < N; ++i)
    {
      mean += sim.run(B);
    }
    mean /= N*1.0;
    BOOST_CHECK_CLOSE(optValue, mean, 1);
  }

  BOOST_AUTO_TEST_CASE(testSimulationAccuraryForImpunc)
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");

    double optValue = 0;
    DynamicPolicy policy;
    DynamicAlgorithm<ImplementationUncertaintyEvaluator>().optimalPolicyAndValue(n, B, policy, optValue);
    
    NullStateVisitor vnull;
    Simulation<DynamicPolicy, NullStateVisitor, true> sim(n, policy, vnull);
    double mean = 0;
    size_t N = 10000L;
    for(size_t i = 0; i < N; ++i)
    {
      mean += sim.run(B);
    }
    mean /= N*1.0;
    BOOST_CHECK_CLOSE(optValue, mean, 1);
  }
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( crashingGame )

  BOOST_AUTO_TEST_CASE(testCrashingGameCorrectnessForFastEvaluator)
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.8/Pat12.rcp");
    
    double optValue = 0;
    DynamicPolicy policy;
    DynamicAlgorithm<CrashingEvaluator>().optimalPolicyAndValue(n, B, policy, optValue);

    double fastMeanVal = FastEvaluator<DynamicPolicy, CrashingEvaluator>(policy).evaluate(n, B);
    BOOST_CHECK_CLOSE(optValue, fastMeanVal, 1e-4);
    BOOST_CHECK_CLOSE(38.5665, fastMeanVal, 1e-4);
  }

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( newStaticHeuristicShouldBeVeryGood )

  BOOST_AUTO_TEST_CASE(testNetworkWithAdvantageForStatic)
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.5/Pat57.rcp");
    
    StaticPolicy staticPolicy;
    double staticValue = staticStochasticPolicy(n, B, staticPolicy);
    
    StaticPolicy heurStaticPolicy;
    double heurStaticValue = staticStochasticPolicyHeuristic<StandardEvaluator>(n, B, heurStaticPolicy);
    
    BOOST_CHECK_CLOSE(staticValue, heurStaticValue, 1e-4);
  }

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( testDeterministicALternatives )

  BOOST_AUTO_TEST_CASE(testSameValueAsMilp)
  {
    const int B = 3;
    Network n;
    n.import("../samples/10-OS-0.5/Pat57.rcp");
    
    StaticPolicy staticPolicy;
    double milpValue = deterministicPolicy(n, B, staticPolicy);

    StaticPolicies sps;
    double dpValue = allOptimalDeterministicPolicies(n, B,sps); 
    BOOST_CHECK_CLOSE(milpValue, dpValue, 1e-4);

    Network& net_ = n;
    
    std::vector<double> oldNus(n.size(), 0);
    BOOST_FOREACH(StaticPolicy dsp, sps)
    {
      std::cout << "Checking static policy ";
      std::copy(dsp._targets.begin(), dsp._targets.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
      std::cout << std::endl;
      vertex_i vi, vi_end;

      for(boost::tie(vi, vi_end) = boost::vertices(n.graph()); vi != vi_end; ++vi)
      {
        oldNus[*vi] = TASK_ATTR(*vi, _nu);
        if(dsp._targets.find(*vi) != dsp._targets.end())
          MUTABLE_TASK_ATTR(*vi, _nu) = TASK_ATTR(*vi, _delta);
      }

      BOOST_CHECK_CLOSE(milpValue, deterministicPolicy(n, 0, staticPolicy), 1e-4);
      for(boost::tie(vi, vi_end) = boost::vertices(n.graph()); vi != vi_end; ++vi)
      {
        MUTABLE_TASK_ATTR(*vi, _nu) = oldNus[*vi];
      }
    }
  }

BOOST_AUTO_TEST_SUITE_END()
