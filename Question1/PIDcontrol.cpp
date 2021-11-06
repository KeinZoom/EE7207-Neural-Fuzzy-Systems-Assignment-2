#include <iostream>
#include <vector>
#include <math.h>
#define PI 3.1415926
#include <fstream>
using namespace std;

ofstream fout_y, fout_theta, fout_u, fout_x;

vector<float> theta_data, y_data, x_data, u_data;

void plant(float u, float &delta_y, float &delta_theta, float y_tar, float theta_tar)
{
    u_data.push_back(u);
    float y,x,theta;
    y = y_data[y_data.size()-1]+0.1*0.5* sin(theta_data[theta_data.size()-1]*PI/180.0f);
    x = x_data[x_data.size()-1]+0.1*0.5* cos(theta_data[theta_data.size()-1]*PI/180.0f);
    y_data.push_back(y);
    x_data.push_back(x);

    theta= theta_data[theta_data.size()-1]+ 0.5* 0.1 * tan(u*PI/180.0f) / 2.5 /PI * 180;
    theta_data.push_back(theta);

    delta_y = y_tar-y;
    delta_theta = theta_tar-theta;
}

float gen_u(float delta_y, float delta_theta)
{
    float u,Ky,Ktheta;
    Ky = 8;
    Ktheta = 4;
    u = Ky*delta_y+Ktheta*delta_theta;

    if(u>30)
    {
        u=30;
    }
    else if(u<-30)
    {
        u=-30;
    }
    return u;
}

int main()
{
    float y_tar=0;
    float theta_tar=0;

    float theta_init = -10;
    float y_init=10;
    float x_init = 50;


    y_data.push_back(y_init);
    x_data.push_back(x_init);
    theta_data.push_back(theta_init);
    
    float delta_y=y_tar-y_init;
    float delta_theta=theta_tar-theta_init;

    while(delta_y*delta_y+delta_theta*delta_theta>0.1)
    {
        plant(gen_u(delta_y,delta_theta),delta_y,delta_theta,y_tar,theta_tar);
        
    }

    fout_y.open("PIDy_data.txt");
    fout_y.precision(10);
    fout_theta.open("PIDtheta_data.txt");
    fout_theta.precision(10);
    fout_u.open("PIDu_data.txt");
    fout_u.precision(10);
    fout_x.open("PIDx_data.txt");
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
    return 0;
}