#pragma once

#include <map>
#include <iostream>
#include <fstream>

class Configurable 
{
public:
  bool loadParamFromFile(const std::string &fileName) 
  {
    std::ifstream in(fileName);

    if (!in.good())
      return false;

    std::string name;
    std::map<const std::string, double *>::iterator res;
    double val;
    bool ret = true;
    while (true)
    {
      in >> name;
      if (in.eof())
        break;

      res = _paramLookup.find(name);
      // can't find item
      if (res == _paramLookup.end()) {
        std::cerr << "unknown param: " << name << " aborting." << std::endl;
        ret = false;
        break;
      }

      in >> val;
      *(res->second) = val;
      //std::cerr << "read " << name << " = " << val << std::endl;
    }

    return ret;
  }

  void printParam(std::ostream &out) const 
  {
    for (auto it = _paramLookup.begin(); it != _paramLookup.end(); it++)
      out << it->first << ": " << *(it->second) << std::endl;
  }
  
protected:
  
  std::map<const std::string, double *> _paramLookup;
  virtual bool _setupParamLookup()=0;

};
