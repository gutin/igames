#include "Network.hpp"
#include "UDCNetwork.hpp"

#include "Task.hpp"
#include "DynamicAlgorithm.hpp"

#include <iostream>

using namespace ig::core;
using namespace std;

int main()
{
  Network n;
  n.import("/Users/gutin/project/rcpdat/6-OS-0.5/Pat2.rcp");

  UDCNetwork unet(n);
  cout << "Done" << endl;

  size_t budget = 1;
  double value = DynamicAlgorithm<StandardEvaluator>::optimalValue(n, budget);
  cout << "Value is " << value << endl;
}
