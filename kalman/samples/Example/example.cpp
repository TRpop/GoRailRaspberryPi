#include <cstdlib>
#include <iostream>
#include <fstream>

#include <cmath>

// Example of Extented Kalman Filter
#include "plane.h"

using namespace std;
using namespace Kalman;


int main() {

	ifstream dataInput;
	ofstream dataOutput;
	std::string tmpStr;

	const unsigned NTRY = 500;
	const unsigned n = 4;	//nb states
	const unsigned m = 2;	//nb measures

	// Matlab format
	selectKVectorContext(createKVectorContext(" ", "[ ", " ];", 4));
	selectKMatrixContext(createKMatrixContext(" ", " ;\n  ", "[ ", " ];", 4));

	cPlaneEKF filter;

	static const double _P0[] = {100.0*100.0, 0.0, 0.0, 0.0,
								 0.0, 10.0*10.0, 0.0, 0.0,
								 0.0, 0.0, 25.0*25.0, 0.0,
								 0.0, 0.0, 0.0, 10.0*10.0}; 

	Vector x(n);
	Matrix P0(n, n, _P0);

	Vector F(NTRY);
	Matrix Measure(m,NTRY);

	dataInput.open("../Matlab/data.m",ifstream::in);
	dataOutput.open("../Matlab/trajectory_udu_load.m", ofstream::out | ofstream::trunc);

	if (dataInput.fail())
	{
		cout<<"Unable to open input file!"<<endl;
		return 0;
	}

	if (dataOutput.fail())
	{
		cout<<"Unable to open output file!"<<endl;
		return 0;
	}

	cout<<"Loading inputs and measures from file <data.m>."<<endl;

	//Read the inputs vector. This vector have been generated by the Matlab script <generation.m>
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>F;

	if (dataInput.fail())
	{
		cout<<"IO error!"<<endl;
		return 0;
	}

	//Output as a Matlab colonn vector.  This will be used by the Matlab script <result.m>
	selectKVectorContext(createKVectorContext(";", "[ ", " ];", 4));


	//Read the measures matrix. This matrix have been generated by the Matlab script <generation.m>
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>Measure;

	if (dataInput.fail())
	{
		cout<<"IO error!"<<endl;
		return 0;
	}

	unsigned i=1, j=1;
	Vector z(m);

	//Initiale estimate
	cout<<"angle: "<<Measure(1,1)<<"rayon: "<<Measure(2,1)<<endl;
	x(1) = cos(Measure(1,1))*Measure(2,1);
	x(2) = 60;
	x(3) = sin(Measure(1,1))*Measure(2,1);
	x(4) = 0;

	filter.init(x, P0);

	cout << "xp(" << ":," << i<<") = " << filter.getX()<<endl;

	dataOutput<<"trajectory_udu(" << ":," << i <<") = " << filter.getX()<<endl;

	for (i = 2; i <= NTRY; ++i) 
	{
		// filter
		for(j = 1; j <= m; j++)
			z(j) = Measure(j,i);

		Vector u(1, F(i));

		filter.step(u, z);

		cout << "xp(" << ":," << i<<") = " << filter.getX()<<endl;
		dataOutput<<"trajectory_udu(" << ":," << i<<") = " << filter.getX()<<endl;
	}


	dataOutput.close();
	dataInput.close();

	return EXIT_SUCCESS;
}
