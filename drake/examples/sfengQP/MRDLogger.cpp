#include "MRDLogger.h"
#include <fstream>
#include <iostream>

// the real max size is MAX_CHANNEL_LENGTH - 1
#define MAX_CHANNEL_LENGTH      3

size_t MRDLogger::maxSize() const { return MAX_CHANNEL_LENGTH - 1; }

MRDLogger::MRDLogger()
{
  _reset();
}

MRDLogger::~MRDLogger()
{

}

void MRDLogger::_reset()
{
  _ptStart = 0;
  _ptEnd = 0;
}

bool MRDLogger::_addChannel(const std::string &name, const std::string &unit, const void *ptr, LoggerDataType type)
{
  if (_channels.find(name) != _channels.end())
    return false;

  _channels[name] = DataChannel();

  _channels[name].name = name;
  _channels[name].unit = unit;
  _channels[name].type = type;
  _channels[name].channel = _channels.size()-1;
  _channels[name].ptr = ptr;
  _channels[name].data.resize(MAX_CHANNEL_LENGTH, 0);

  _outputOrder.push_back(&_channels[name]);
  //std::cout << _channels.size() << " " << _outputOrder.size() << std::endl;

  return true;
}

bool MRDLogger::readFromFile(const std::string &name)
{
  std::ifstream in;
  in.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  
  size_t n_channels, n_points, tot;
  float tmp_data;
  char *ptr = (char *)&tmp_data;

  try {
    _reset();
    in.open(name);
    // read header
    in >> n_points;
    in >> n_channels;
    in >> tot;
    in >> _freq;

    std::vector<std::string> names(n_channels);
    std::vector<std::string> units(n_channels);

    // read the channel names and units
    std::string tmp;
    for (size_t i = 0; i < n_channels; i++) {
      in >> tmp;
      names[i] = tmp;
      in >> tmp;
      units[i] = tmp;
    }

    // get 3 \n
    in.get();
    in.get();
    in.get();

    // read data
    for (size_t t = 0; t < n_points; t++) {
      for (size_t i = 0; i < n_channels; i++) {
        ptr[3] = in.get();
        ptr[2] = in.get();
        ptr[1] = in.get();
        ptr[0] = in.get();

        auto it = _channels.find(names[i]);
        if (it != _channels.end()) {
          //std::cout << "read: " << it->first << " " << tmp_data << std::endl;
          it->second.data[_ptEnd] = tmp_data;
        }
      }

      _ptEnd = (_ptEnd+1) % MAX_CHANNEL_LENGTH;
      if (_ptEnd == _ptStart)
        _ptStart = (_ptStart+1) % MAX_CHANNEL_LENGTH;

      //std::cout << "read: " << tmp_data << std::endl;
      //std::cout << "start idx: " << _ptStart << " end idx: " << _ptEnd << std::endl;
    }
  }
  catch (std::ifstream::failure e) {
    std::cerr << "error when parsing data\n";
    return false;
  }

  return true;
}

bool MRDLogger::writeToFile(const std::string &name) const
{
  if (size() == 0)
    return true;

  std::ofstream out;
  out.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  
  try {
    out.open(name.c_str(), std::ofstream::out);
    // write header
    out << this->size() << " " << _channels.size() << " " << _channels.size()*_ptEnd << " " << _freq << std::endl;

    // write names and units
    for (auto it = _outputOrder.begin(); it != _outputOrder.end(); it++) {
      out << (*it)->name << " " << (*it)->unit << std::endl;
    }

    // write 2 empty line
    out << std::endl << std::endl;

    // write data
    size_t i = _ptStart;
    size_t len = 0;
    while(true) {
      for (auto it = _outputOrder.begin(); it != _outputOrder.end(); it++) {
        len = (*it)->data.size();
        char *ptr = (char *)(&((*it)->data.at(i)));
        out.put(ptr[3]);
        out.put(ptr[2]);
        out.put(ptr[1]);
        out.put(ptr[0]);  
        
        //std::cout << "write: " << (*it)->name << " " << (*it)->data.at(i) << std::endl;
      }
      //std::cout << "write idx: " << i << " end: " << _ptEnd << std::endl;

      i++;
      if (i == len)
        i = 0;
      if (i == _ptEnd)
        break;
    }

    out.close();
  }
  catch (std::ofstream::failure e) {
    std::cerr << "error when writing data\n";
  }

  return true;
}

size_t MRDLogger::size() const
{
  if (_ptEnd >= _ptStart)
    return _ptEnd - _ptStart;
  else
    return _ptEnd + MAX_CHANNEL_LENGTH - _ptStart;
}

void MRDLogger::saveData()
{
  for (auto it = _channels.begin(); it != _channels.end(); it++) {
    switch (it->second.type) {
      case LOGGER_DATA_TYPE_BOOL:
        it->second.data[_ptEnd] = (float) *(bool *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_CHAR:
        it->second.data[_ptEnd] = (float) *(char *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_INT:
        it->second.data[_ptEnd] = (float) *(int *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_FLOAT:
        it->second.data[_ptEnd] = (float) *(float *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_DOUBLE:
        it->second.data[_ptEnd] = (float) *(double *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_LONG:
        it->second.data[_ptEnd] = (float) *(long *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_UCHAR:
        it->second.data[_ptEnd] = (float) *(unsigned char *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_UINT:
        it->second.data[_ptEnd] = (float) *(unsigned int *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_ULONG:
        it->second.data[_ptEnd] = (float) *(unsigned long *)(it->second.ptr);
        break;

      default:
        continue;
    }
  }
  
  _ptEnd++;
  if (_ptEnd >= MAX_CHANNEL_LENGTH)
    _ptEnd = 0;
  if (_ptEnd == _ptStart)
    _ptStart++;
  if (_ptStart >= MAX_CHANNEL_LENGTH)
    _ptStart = 0;
  
  //std::cout << "start/end " << _ptStart << " " << _ptEnd << std::endl;
}

void MRDLogger::popData()
{
  if (!hasMoreData())
    return;

  for (auto it = _channels.begin(); it != _channels.end(); it++) {
    switch (it->second.type) {
      case LOGGER_DATA_TYPE_BOOL:
        *(bool *)(it->second.ptr) = (bool)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_CHAR:
        *(char *)(it->second.ptr) = (char)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_INT:
        *(int *)(it->second.ptr) = (int)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_FLOAT:
        *(float *)(it->second.ptr) = (float)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_DOUBLE:
        *(double *)(it->second.ptr) = (double)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_LONG:
        *(long *)(it->second.ptr) = (long)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_UCHAR:
        *(unsigned char *)(it->second.ptr) = (unsigned char)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_UINT:
        *(unsigned int *)(it->second.ptr) = (unsigned int)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_ULONG:
        *(unsigned long *)(it->second.ptr) = (unsigned long)it->second.data[_ptStart];
        break;

      default:
        continue;
    }
  }

  _ptStart = (_ptStart+1) % MAX_CHANNEL_LENGTH;
}
