#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include "mypchip.h"
using namespace std;
// g++ readfile.cpp -lstdc++
// vector<vector<double>> delp1;
// vector<vector<double>> delp2;
// vector<vector<double>> delp3;
// vector<vector<double>> delp4;
// vector<vector<double>> delp5;
// vector<vector<double>> delp6;

int readdoubles7(std::string filename, std::vector<vector<double>> &scores)
{
   std::ifstream ifile(filename, std::ios::in);

    //check to see that the file was opened correctly:
        if (!ifile.is_open()) {
         std::cerr << "There was a problem opening the input file!\n";
         exit(1);//exit or do additional error checking
                   }
       double num = 0.0;
       vector<double> v(7);
    //keep storing values from the text file so long as data exists:
       while (ifile >> v[0]>>v[1]>>v[2]>>v[3]>>v[4]>>v[5]>>v[6]) {
        scores.push_back(v);
            }
    //verify that the scores were stored correctly:
       for (int i = 0; i < scores.size(); ++i) {
       //  std::cout << scores[i] << std::endl;
                                                  }
return 0;
}
int readdoubles4(std::string filename, std::vector<vector<double>> &scores)
{
   std::ifstream ifile(filename, std::ios::in);
    //check to see that the file was opened correctly:
        if (!ifile.is_open()) {
         std::cerr << "There was a problem opening the input file!\n";
         exit(1);//exit or do additional error checking
                   }
       double num = 0.0;
       vector<double> v(4);
    //keep storing values from the text file so long as data exists:
       while (ifile >> v[0]>>v[1]>>v[2]>>v[3]) {
        scores.push_back(v);
            }
    //verify that the scores were stored correctly:
       for (int i = 0; i < scores.size(); ++i) {
       //  std::cout << scores[i] << std::endl;
                                                  }
return 0;
}

int readdelp(string filename, vector<vector<double>> &scores)
{
   std::ifstream ifile(filename, std::ios::in);

    //check to see that the file was opened correctly:
        if (!ifile.is_open()) {
         std::cerr << "There was a problem opening the input file!\n";
         exit(1);//exit or do additional error checking
                   }
       double num1,num2;int i=0;
       vector<double> v(2);
    //keep storing values from the text file so long as data exists:
       while (ifile >> v[0]>>v[1]) {
        scores.push_back(v);
            }
    //verify that the scores were stored correctly:
  //   for (vector<double> v:scores) { 
  //          cout << v[0] << " "<<v[1]<<endl; 
 //   } 
return 0;
} 
int devidedelp( vector<vector<double>> &scores1, 
		vector<vector<double>> &scores2,
		vector<vector<double>> &scores3,
		vector<vector<double>> &scores4,
		vector<vector<double>> &scores5,
		vector<vector<double>> &scores6){
vector<vector<double>> scores;
       vector<double> v(2);
int i=0;
readdelp("src/delp.txt",scores);
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
scores1.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
scores2.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
scores3.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
scores4.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
scores5.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(i<scores.size()){
v[0]=scores[i][0];v[1]=scores[i][1];
scores6.push_back(v);
i++;
};
} 
vector<vector<double>> getDelpCurveFromFile( string filename,int col ){
vector<vector<double>> scores;
vector<vector<double>> ret;
       vector<double> v(2);
int i=0;
readdelp(filename,scores);
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
if (col==0) ret.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
if (col==1) ret.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
if (col==2) ret.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
if (col==3) ret.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(scores[i][0]!=0){
v[0]=scores[i][0];v[1]=scores[i][1];
if (col==4) ret.push_back(v);
i++;
};
i++;//cout<<1<<endl;
while(i<scores.size()){
v[0]=scores[i][0];v[1]=scores[i][1];
if (col==5) ret.push_back(v);
i++;
};
return ret;
} 

/*int main ()
{  
 vector<vector<double>> scores1;
 vector<vector<double>> scores2;
 vector<vector<double>> scores3;
 vector<vector<double>> scores4;
 vector<vector<double>> scores5;
 vector<vector<double>> scores6;
devidedelp( scores1,scores2,scores3,scores4,scores5,scores6);
 cout<<pchiptx(scores2,100)<<endl;
//cout<< scores1[scores1.size()-1][0] << ":"<<scores1[scores1.size()-1][1]<< endl;
return 0;
}*/
