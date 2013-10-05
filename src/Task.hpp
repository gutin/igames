#ifndef TASK_H 
#define TASK_H

namespace ig { namespace core {

struct Task
{
  double expNormal() const { return 1.0 / _nu; }
  double expDelayed() const { return 1.0 / _delta; }

  double _nu, _delta;
  double _probDelaySuccess;
};

}}

#endif // TASK_H
