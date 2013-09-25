#ifndef TASK_H 
#define TASK_H

namespace ig { namespace core {

class Task
{
public:
  Task(double nu_, double delta_, int index_) : _nu(nu_), _delta(delta_), _index(index_) {}
  Task() {}
  void setFrom(const Task&);
  
  double delta() const { return _delta; }
  double nu() const { return _nu; }
  double expNormal() const { return 1.0 / _nu; }
  double expDelayed() const { return 1.0 / _delta; }
  int index() const { return _index; }

  bool operator==(const Task& other) const;
        
  double _nu, _delta;
  int _index;
};

}}

#endif // TASK_H
