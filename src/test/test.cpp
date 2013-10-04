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
    DynamicAlgorithm<StandardEvaluator>::optimalPolicyAndValue(n, B, policy, optValue);

    double workedOutVal = StandardDynamicEvaluator().evaluate(n, B, policy);

    BOOST_CHECK_CLOSE(optValue, workedOutVal, 1e-05);
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
      optValue = DynamicAlgorithm<StandardEvaluator>::execute(n, B, psp);
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
    DynamicAlgorithm<StandardEvaluator>::optimalPolicyAndValue(n, B, policy, optValue);
    
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
    DynamicAlgorithm<ImplementationUncertaintyEvaluator>::optimalPolicyAndValue(n, B, policy, optValue);
    
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
