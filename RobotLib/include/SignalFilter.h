#ifndef SIGNALFILTER_H_
#define SIGNALFILTER_H_

#ifdef _cplusplus
extern "C" {
#endif

#define PI 3.1415926

int LowPass_order1(double f, double Ts, double *y, double *u);

int LowPass_order2(double f, double b, double Ts, double *y, double *u);

#ifdef _cplusplus
}
#endif

#endif /* SIGNALFILTER_H_ */