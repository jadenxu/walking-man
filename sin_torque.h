#include<cmath>
using namespace std;

#ifndef PI
#define PI 3.141592
#endif

class Torque
{
public:
	Torque(){};
	Torque(double A, double T,double phi, double C)
	{
		this->A = A;
		this->T = T;
		this->phi = phi;
		this->C = C;
	}
	double calculate(double t)
	{
		double res = A * sin(2 * PI * t / T + phi) + C;
		return res;
	}
	double A;
	double T;
	double phi;
	double C;
};
