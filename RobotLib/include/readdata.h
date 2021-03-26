/*
 * readdata.h
 *
 *  Created on: 2020-3-25
 *      Author: hanbing
 */

#ifndef READDATA_H_
#define READDATA_H_

#ifdef _cplusplus
extern "C" {
#endif

int getDataNum_points(const char *ininame);

int ReadOffset_XY_Fromfile(double **rbufDou, int *rbufInt, int *m_points, const char *ininame);

int getDataName_points(char *ininame, int n, char *dataname);

int getpoints_file(char *ininame, char *J, double *points);


#ifdef _cplusplus
}
#endif

#endif /* READDATA_H_ */
