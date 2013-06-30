/*
 * libDrakeDebugMex.cpp
 *
 *  Created on: Jun 23, 2013
 *      Author: russt
 */

#include <matrix.h>
#include <mat.h>


mxArray *mxGetProperty(const mxArray *pa, mwIndex index, const char *propname)
{
	return mxGetField(pa,index,propname);
}


