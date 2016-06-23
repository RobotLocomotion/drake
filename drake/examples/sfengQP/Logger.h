#ifndef __LOGGER_H
#define __LOGGER_H

#include <stdio.h>
#include <string.h>
#include <eigen3/Eigen/Geometry>
#include <typeinfo>
#include "mrdplot.h"

#define LOGGER_MAX_CHANNELS 2000 
#define LOGGER_MAX_CHARS		100
#define LOGGER_MAX_QUAT			100

enum LoggerDataType {
	LOOGER_DATA_TYPE_UNDEF = 0,
	LOGGER_DATA_TYPE_DOUBLE,
	LOGGER_DATA_TYPE_FLOAT,
	LOGGER_DATA_TYPE_INT,
	LOGGER_DATA_TYPE_LONG,
	LOGGER_DATA_TYPE_BOOL,
	LOGGER_DATA_TYPE_ULONG,
	LOGGER_DATA_TYPE_UINT
};

class DataPoint 
{
	public:
		char units[LOGGER_MAX_CHARS];
		char names[LOGGER_MAX_CHARS];
		char data_type;
		int channel;			// corresponds to mrdplot channel
		const void *ptr;  // ptr to data

		DataPoint() 
		{
			data_type = LOOGER_DATA_TYPE_UNDEF;
			ptr = NULL;
			channel = -1;
		}
};

class Logger {
  protected:
    int _ctr;
		bool _inited;
    int _nPoints;
    int _nChannels;
    int _myIdx;
    float *_data;
    float _frequency;
    DataPoint _datapoints[LOGGER_MAX_CHANNELS]; 
    Eigen::Quaterniond const *_qPtr[LOGGER_MAX_QUAT];
    double _EAbuff[3*LOGGER_MAX_QUAT];
    int _nQuat;
    bool _recorded;
    
    void init_(double timestep);
  public:
    Logger();
    ~Logger();

    void add_quat(const std::string &names, const Eigen::Quaterniond *q);
    template<typename T> bool add_datapoint(const std::string &names, const char *units, const T *ptr)
		{
			if (_nChannels >= LOGGER_MAX_CHANNELS)
				return false;

			DataPoint *dptr = &_datapoints[_nChannels];
			strncpy(dptr->names,names.c_str(),LOGGER_MAX_CHARS-1);
			dptr->names[LOGGER_MAX_CHARS-1] = '\0';
			strncpy(dptr->units,units,LOGGER_MAX_CHARS-1);
			dptr->units[LOGGER_MAX_CHARS-1] = '\0';

			_datapoints[_nChannels].ptr = (const void *)ptr;
			if (typeid(T) == typeid(bool))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_BOOL;
			else if (typeid(T) == typeid(double))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_DOUBLE;
			else if (typeid(T) == typeid(float))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_FLOAT;
			else if (typeid(T) == typeid(int))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_INT;
			else if (typeid(T) == typeid(long))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_LONG;
			else if (typeid(T) == typeid(unsigned int))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_UINT;
			else if (typeid(T) == typeid(unsigned long))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_ULONG; 
			else {
				fprintf(stdout, "Logger: adding unknown data type, name: %s\n", names.c_str());
				return false;
			}

			_nChannels++; 

			return true;
		}

    
    ///////////////////////////////////////////
    // eric debugging tool
    static double tmpOut[20];
    static int errCode;
    static void setErrEW(int err);
    static void setTmpOut(int ind, double val);
    void addEWstatic();
    ///////////////////////////////////////////
    
    virtual void init(double timestep)=0;
    virtual void saveData()=0;
		
		inline bool hasInited() const { return _inited; }
};

class BatchLogger : public Logger {
  public:
    BatchLogger() {;}
    ~BatchLogger() {;}
    void init(double timestep);
    void saveData();
    void writeToMRDPLOT(const std::string &prefix);
};


class DataParser {
  private:
    MRDPLOT_DATA *_data;
    DataPoint _datapoints[LOGGER_MAX_CHANNELS];
		int _tIdx;
    int _nChannels;
  
  public:
    DataParser();
    ~DataParser();

		inline int getIdx() const { return _tIdx; }
    inline int size() { if (_data) return _data->n_points; }
    bool load(const char *name);
    
    void add_quat(const std::string &names, const Eigen::Quaterniond *q);
		template<typename T> bool add_datapoint(const std::string &names, const T *ptr)
		{
			int idx = find_channel(names.c_str(), _data);
      
			if (idx == -1 || idx >= LOGGER_MAX_CHANNELS || _nChannels >= LOGGER_MAX_CHANNELS)
				return false;

			_datapoints[_nChannels].channel = idx;
			_datapoints[_nChannels].ptr = (const void *)ptr;
			if (typeid(T) == typeid(bool))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_BOOL;
			else if (typeid(T) == typeid(double))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_DOUBLE;
			else if (typeid(T) == typeid(float))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_FLOAT;
			else if (typeid(T) == typeid(int))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_INT;
			else if (typeid(T) == typeid(long))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_LONG;
			else if (typeid(T) == typeid(unsigned int))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_UINT;
			else if (typeid(T) == typeid(unsigned long))
				_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_ULONG;  
			else {
				fprintf(stdout, "Parser: adding unknown data type, name: %s\n", names.c_str());
				return false;
			}
			_nChannels++;
			return true; 
		}

		bool setIdx(int idx);
		bool readData(int idx = -1);
}; 

#endif
