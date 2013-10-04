#include "Network.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <ctime>

using namespace std;

namespace ig { namespace core {

void Network::connect(const vertex_t& s_, const vertex_t& t_)
{
  boost::add_edge(s_, t_, _g);
}

vertex_t Network::add(const Task& t_)
{
  vertex_t u = boost::add_vertex(_g);
  _g[u]._index = t_._index;
  _g[u]._nu = t_._nu;
  _g[u]._delta = t_._delta;
  return u;
}

size_t Network::size() const
{
  return boost::num_vertices(_g);
}

void Network::initSecondary()
{
  _successorbs.clear();
  _successorbs.reserve(size());
  _predbs.clear();
  _predbs.reserve(size());
  for(size_t i = 0; i < size(); ++i)
  {
    _successorbs.push_back(0);
    _predbs.push_back(0);
  }
  vertex_i vi, vi_end;
  boost::tie(vi, vi_end) = boost::vertices(_g);
  for(; vi != vi_end; ++vi)
  {
    ine_i ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(*vi, _g);
    for(; ei != ei_end; ++ei)
    {
      vertex_t pred = boost::source(*ei, _g);
      _successorbs[pred].set(*vi);
      _predbs[*vi].set(pred);
    }
  }
}

vertex_t Network::start() const
{
  return _start; 
}

vertex_t Network::end() const
{
  return _end;
}

Network::Network()
{
}

bool Network::import(const std::string& file_, bool delaysFromFile_)
{
  std::ifstream ifs(file_.c_str(), std::ifstream::in);
  std::string line;
  int count(0), activityName(0);
  double duration;
  
  std::srand(0);//deterministic stream of random nums

  vertex_t last;
  while(std::getline(ifs, line))
  {
    if(count >= 4)
    {
      std::istringstream iss(line);
      if(iss >> duration)
      {
        double delayedDuration = 0;
        if(delaysFromFile_)
        {
          if(!(iss >> delayedDuration) || delayedDuration < duration)
          {
            std::cerr << "Supposed to use delayed duration from file but the data is incorrect" << std::endl;
            std::cerr << "Normal = " << duration << "; delayed = " << delayedDuration << std::endl;
            return false;
          }
        }
        else
        {
          // if we dont take from the file the default is to double the delayedDuration
          delayedDuration = 2*duration;
        }
        Task t((1/duration), (1/delayedDuration), activityName);
        t._probDelaySuccess = std::rand() / double(RAND_MAX); 
        last = add(t);
        std::cout << "Adding task [" << last << "] with duration [" << duration << "]"
                  << " delayedDuration [" << delayedDuration << "]. Delay success prob [" << t._probDelaySuccess  << "]" << std::endl;
        if(count == 4) _start = last;
        ++activityName;
      }
      else
      {
        //error 
        std::cerr << "Error no duration info avail" << std::endl;
        return false;
      }
    }
    ++count;
  }
  _end = last;
  ifs.close();

  ifs.open(file_.c_str(), std::ifstream::in);
  count=0;
  activityName = 0; 
  while(std::getline(ifs, line))
  { 
    if( count >= 4)
    {
      std::istringstream issl(line);
      vertex_i current = findFromIndex(activityName);   
      ++activityName;

      int next, intCount=0;
      while(issl >> next)
      {
        if(intCount++ <= 5) continue;
        vertex_i nextV = findFromIndex(next-1);
        connect(*current, *nextV);
        std::cout << "Connected tasks with idxs " << _g[*current].index() << ", " << _g[*nextV].index() << std::endl;
      }
    }
    count++;
  }
  ifs.close();
  initSecondary();
  return true;
}


vertex_i Network::findFromIndex(int i_) const
{
  vertex_i vi, vi_end;
  for (boost::tie(vi, vi_end) = boost::vertices(_g); vi != vi_end; ++vi)
  {
    if(_g[*vi].index() == i_) return vi;
  }
  return vi_end;
}

double Network::orderStrength() const
{
  //TODO: implement this
  return 0.8;
}

}}
