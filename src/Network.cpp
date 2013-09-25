#include "Network.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <sstream>

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

inline size_t Network::size() const
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
      _successorbs[pred] |= (1 << *vi);
      _predbs[*vi] |= (1 << pred);
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

bool Network::import(const std::string& file_)
{
  std::ifstream ifs(file_.c_str(), std::ifstream::in);
  std::string line;
  int count(0), activityName(0);
  double duration;

  vertex_t last;
  while(std::getline(ifs, line))
  {
    if(count >= 4)
    {
      std::istringstream iss(line);
      if(iss >> duration)
      {
        Task t((1/duration), (1/(2*duration)), activityName);
        last = add(t);
        std::cout << "Adding task [" << last << "] with duration [" << duration << "]" << std::endl;
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

}}
