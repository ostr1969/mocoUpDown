#include <OpenSim/OpenSim.h>
#include  "mypchip.h"
#include  <OpenSim/Common/readxy.h>
#include <iostream>
	std::vector<vector<double>> delpcurve;
ofstream Log("results/trypc.Log", ofstream::out);
int main(){

	readxy("src/delp1.txt",delpcurve);
	int L=delpcurve.size()-1;int n=300;
        cout<<delpcurve[0][0]<<","<<delpcurve[0][1]<<endl;
	cout<<delpcurve[L][0]<<","<<delpcurve[L][1]<<endl;
	double x=-2.2,d=(-0.4+2.2)/n;
        for (int i=0; i<n; i++){
	x+=d;
	double optimal=limitedPchip(delpcurve,x);
        cout<<x<<"," <<optimal<<endl;
        Log<<x<<"," <<optimal<<endl;
	}
}
