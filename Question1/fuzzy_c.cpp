#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Fuzzy_controller.h"
#include <fstream>
#define PI 3.1415926
using namespace std;

ofstream fout_y, fout_theta, fout_u, fout_x;

int main()
{
float y_target=0;
float theta_target=0;

float x_actual=50;
float y_actual=10;
float theta_actual=-10;
int iter = 2000;

float u=0;

string ruleMatrix[7][5]=
{
{"PB","PB","PM","PM","PS"},
{"PB","PB","PM","PS","NS"},
{"PB","PM","PS","NS","NM"},
{"PM","PM","ZE","NM","NM"},
{"PM","PS","NS","NM","NB"},
{"PB","NS","NM","NB","NB"},
{"NS","NM","NM","NB","NB"}
};//模糊规则表

float y_mf_paras[15]={
    -100,-40,-12,
    -20,-10,0,
    -5,0,5,
    0,10,20,
    12,40,100};
//y的隶属度函数参数 3个数据为一组

float theta_mf_paras[21]={
    -180,-125,-80,
    -100,-60,-40,
    -50,-25,0,
    -20,0,20,
    0,25,50,
    40,60,100,
    80,125,180};//误差变化率de的模糊隶属度函数参数

float u_mf_paras[21]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};//输出量u的隶属度函数参数
Fuzzy_controller fuzzy(100, 180, 30);//控制器的最大值

fuzzy.setMf(y_mf_paras,theta_mf_paras,u_mf_paras);//设定模糊隶属度函数

fuzzy.setRule(ruleMatrix);//设定模糊规则

cout<<"num y_target y_actual theta_target theta_actual"<<endl;

float loss = y_actual*y_actual + theta_actual*theta_actual;
float min_y = y_actual;
float min_theta = theta_actual;
int min_y_index, min_theta_index;
vector<float> y_data,theta_data,u_data, x_data;
y_data.push_back(y_actual);
theta_data.push_back(theta_actual);
x_data.push_back(x_actual); 
for(int i=0;i<iter;i++)
{
    
    u=fuzzy.realize(y_target,theta_target,y_actual, theta_actual);
    
    // if(i%100==0)
    // {cout<< i+1 <<" "<<u <<" "<<y_target<<" "<<y_actual<<" "<< theta_target<<" "<<theta_actual<<endl;}
    
    y_actual+= 0.5 * 0.1 * sin(theta_actual*PI/180.0f);
    x_actual+= 0.5 * 0.1 * cos(theta_actual*PI/180.0f);
    theta_actual+= 0.5* 0.1 * tan(u*PI/180.0f) / 2.5 /PI * 180;
    if (abs(y_actual) < abs(min_y))
    {
        min_y = y_actual;
        min_y_index = i+1;
    }
    if (abs(theta_actual) < abs(min_theta))
    {
        min_theta = theta_actual;
        min_theta_index = i+1;
    }
    if( abs(y_actual)<1.0 && abs(theta_actual)<0.01)
    {
    cout<< i+1 <<" "<<u <<" "<<y_target<<" "<<y_actual<<" "<< theta_target<<" "<<theta_actual<<endl;
    }
    y_data.push_back(y_actual);
    theta_data.push_back(theta_actual);
    u_data.push_back(u);
    x_data.push_back(x_actual);    
}
fout_y.open("y_data.txt");
fout_y.precision(10);
fout_theta.open("theta_data.txt");
fout_theta.precision(10);
fout_u.open("u_data.txt");
fout_u.precision(10);
fout_x.open("x_data.txt");
fout_x.precision(10);

for( int i = 0; i<y_data.size(); i++)
{
    fout_y<<i+1<<" "<<y_data[i]<<endl;
    fout_theta<<i+1<<" "<<theta_data[i]<<endl;
    fout_u<<i+1<<" "<<u_data[i]<<endl;
    fout_x<<i+1<<" "<<x_data[i]<<endl;
}
fout_y.close();
fout_theta.close();
fout_u.close();
fout_x.close();
// fuzzy.showInfo();
return 0;
}