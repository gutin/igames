#ifndef TASK_H 
#define TASK_H

namespace ig { namespace core {

struct Task
{
  /*
   * Expected normal and delayed durations (assuming exponential distros)
   */
  double expNormal() const { return 1.0 / _nu; }
  double expDelayed() const { return 1.0 / _delta; }

  /*
   * Normal and delayed rates
   */
  double _nu, _delta;

  /*
   *Probability interdiction is successful
   */
  double _probDelaySuccess;

  /*
   * Unit cost of crashing that task by the single renewable resource
   */
  double _crashingCost;

  /*
   * Minimum investment of the single renewable resource into this task
   */
  const static double MIN_INVESTMENT = 1;
};

}}

#endif // TASK_H
