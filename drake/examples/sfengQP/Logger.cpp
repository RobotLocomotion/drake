#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include "Logger.h"
#include "mrdplot.h"
//#include <cmu_walk/Utils.hpp>
//#include <cmu_walk/Eigen_utils.hpp>

// If running on the dev boards and you want to lock memory change
// this to 50000.  500000 results in more than 8gigs - the memory size
// on the dev boards
#define LOGGER_MAX_N_POINTS 100000

Logger::Logger() {
  _data = NULL;
  _nPoints = 0;
  _myIdx = 0;
  _frequency = 0;
  _nChannels = 0;
  _inited = false;

  _nQuat = 0;
  _recorded = false;
}

Logger::~Logger()
{
  if (_data)
    delete []_data;
  _data = NULL;
}

void Logger::init_(double timestep)
{
  _ctr = 0;
  _inited = true;
  _myIdx = 0;
  _nPoints = 0;
  _nChannels = 0;
  _data = new float[LOGGER_MAX_N_POINTS*LOGGER_MAX_CHANNELS];

  //_frequency = 333;
  _frequency = 1./timestep;
  add_datapoint("ctr","-",&_ctr);
}

void Logger::add_quat(const std::string &names, const Eigen::Quaterniond *q) {
  if (_nQuat < LOGGER_MAX_QUAT) {
    _qPtr[_nQuat] = q;
    char name[100];
    sprintf(name, "%s[RR]",names.c_str());
    add_datapoint(name, "r", &(_EAbuff[_nQuat*3]));
    sprintf(name, "%s[PP]",names.c_str());
    add_datapoint(name, "r", &(_EAbuff[_nQuat*3+1]));
    sprintf(name, "%s[YY]",names.c_str());
    add_datapoint(name, "r", &(_EAbuff[_nQuat*3+2]));

    sprintf(name, "%s[X]",names.c_str());
    add_datapoint(name, "-", q->coeffs().data());
    sprintf(name, "%s[Y]",names.c_str());
    add_datapoint(name, "-", q->coeffs().data()+1);
    sprintf(name, "%s[Z]",names.c_str());
    add_datapoint(name, "-", q->coeffs().data()+2); 
    sprintf(name, "%s[W]",names.c_str());
    add_datapoint(name, "-", q->coeffs().data()+3); 
    _nQuat++;
  }
}

//////////////////////////////////////////////////////////////////////
void BatchLogger::init(double timestep)
{
  init_(timestep);
}

bool noInitSent = false;

void BatchLogger::saveData()
{
  //static bool err = false;

  if(_recorded)	return;		//stop wasting your time

  for(int i = 0; i < _nQuat; i++) {
    //quat2EA(*(_qPtr[i]), &(_EAbuff[i*3]));
  }

  const void *ptr;

  if (!_inited) {
    if(!noInitSent) {
      fprintf(stdout, "logger not inited.\n");
      noInitSent = true;
    }
    return;
  }

  for(int i = 0; i < _nChannels; i++) {
    ptr = _datapoints[i].ptr;
    if (_datapoints[i].data_type == LOGGER_DATA_TYPE_DOUBLE)
      _data[_myIdx+i] = (float)(*((const double *)ptr));
    else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_FLOAT)
      _data[_myIdx+i] = (float)(*((const float *)ptr));
    else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_INT)
      _data[_myIdx+i] = (float)(*((const int *)ptr));
    else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_LONG)
      _data[_myIdx+i] = (float)(*((const long *)ptr));
    else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_BOOL) {
      if(*((bool *)ptr))
        _data[_myIdx+i]	= 1;
      else
        _data[_myIdx+i]	= 0;
    }
  }
  _myIdx += _nChannels;
  _nPoints++;
  _ctr++;

  if (_myIdx / _nChannels >= LOGGER_MAX_N_POINTS) {
    fprintf(stdout, "logger loop back.\n"); 
    //_nPoints = 0;
    _myIdx = 0;
  }
  //clamp(_nPoints, 0, LOGGER_MAX_N_POINTS);
  if (_nPoints >= LOGGER_MAX_N_POINTS)
    _nPoints = LOGGER_MAX_N_POINTS;
} 

void BatchLogger::writeToMRDPLOT(const std::string &prefix)
{
  if (!_inited || _nPoints == 0)
    return;
  _recorded = true;
  MRDPLOT_DATA *d;

  d = malloc_mrdplot_data( 0, 0 );
  d->filename = generate_file_name(prefix.c_str());
  d->n_channels = _nChannels;
  d->n_points = _nPoints;
  d->total_n_numbers = d->n_channels*d->n_points;
  d->frequency = _frequency;
  d->data = _data;

  fprintf(stdout, "%s SAVING DATA ..... \n", d->filename);

  d->names = new char*[_nChannels];
  d->units = new char*[_nChannels];

  for (int i = 0; i < _nChannels; i++) {
    d->names[i] = new char[LOGGER_MAX_CHARS];
    d->units[i] = new char[LOGGER_MAX_CHARS];

    strcpy(d->names[i], _datapoints[i].names);
    strcpy(d->units[i], _datapoints[i].units);
  }

  write_mrdplot_file( d );

  for (int i = 0; i < _nChannels; i++) {
    delete []d->names[i];
    delete []d->units[i];
  }
  delete []d->names;
  delete []d->units;

  fprintf(stdout, "%s SAVED DATA\n", d->filename);
  free(d);
}



///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
DataParser::DataParser()
{
  _data = NULL;
  _tIdx = 0;
}

DataParser::~DataParser()
{
  if (_data)  
    free(_data);
  _data = NULL;
}

bool DataParser::load(const char *name)
{
  _data = read_mrdplot(name);
  return (_data != false);
}

void DataParser::add_quat(const std::string &names, const Eigen::Quaterniond *q)
{
  char name[100];
  sprintf(name,"%s[X]",names.c_str());
  add_datapoint(name, q->coeffs().data());
  sprintf(name,"%s[Y]",names.c_str());
  add_datapoint(name, q->coeffs().data()+1);
  sprintf(name,"%s[Z]",names.c_str());
  add_datapoint(name, q->coeffs().data()+2); 
  sprintf(name,"%s[W]",names.c_str());
  add_datapoint(name, q->coeffs().data()+3); 
}

bool DataParser::setIdx(int idx)
{
  if (idx < 0 || idx >= _data->n_points)	
    return false;
  _tIdx = idx;
  return true;
}

bool DataParser::readData(int idx)
{
  int trace, t;
  if (idx == -1) {
    t = _tIdx;
    _tIdx++;
  }
  else
    t = idx;

  if (t >= _data->n_points)
    return false;  

  for (int i = 0; i < _nChannels; i++) {
    trace = _datapoints[i].channel;
    //printf("%d %d\n", trace, _datapoints[i].channel);
    assert(trace >= 0 && trace < _data->n_channels);
    assert(t >= 0 && t < _data->n_points);

    switch (_datapoints[i].data_type) {
      case LOGGER_DATA_TYPE_DOUBLE:
        *(double *)_datapoints[i].ptr = (double)_data->data[_data->n_channels*t+trace];
        break;
      case LOGGER_DATA_TYPE_FLOAT:
        *(float *)_datapoints[i].ptr = (float)_data->data[_data->n_channels*t+trace];
        break;
      case LOGGER_DATA_TYPE_INT:
        *(int *)_datapoints[i].ptr = (int)_data->data[_data->n_channels*t+trace];
        break;
      case LOGGER_DATA_TYPE_LONG:
        *(long *)_datapoints[i].ptr = (long)_data->data[_data->n_channels*t+trace];
        break;
      case LOGGER_DATA_TYPE_BOOL:
        *(bool *)_datapoints[i].ptr = (int)_data->data[_data->n_channels*t+trace];
        break;
    }
  }
  return true;
}
