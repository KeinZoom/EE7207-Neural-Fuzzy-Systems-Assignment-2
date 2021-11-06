#include <iostream>
#include <string>
#include <vector>
using namespace std;

class Fuzzy_controller
{
public:
    const static int M=7, N=5;//定义量化论域模糊子集的个数
private:
    float y_target;//系统的控制目标
    float y_actual;//采样获得的实际值
    float theta_target;
    float theta_actual;

    float y_max;  //ymax
    float theta_max; //theta上限
    float u_max;  //输出的上限

    string rule[M][N];//模糊规则表
    string mf_t_y;   //y的隶属度函数类型
    string mf_t_theta;  //theta的隶属度函数类型

    float *y_mf_paras; //y的隶属度函数的参数
    float *theta_mf_paras;//theta隶属度函数的参数
    float *u_mf_paras;
public:
    Fuzzy_controller(float y_max,float theta_max,float u_max);
    ~Fuzzy_controller();
    float trimf(float x,float a,float b,float c);          //三角隶属度函数
    float L_trapmf(float x,float a,float b,float c); //left梯形隶属度函数
    float R_trapmf(float x,float a,float b,float c);
    //设置模糊隶属度函数的参数
    void setMf(float *y_mf,
                float *theta_mf,
                float *u_mf);

    void setRule(string rulelist[M][N]);                          //设置模糊规则
    float realize(float y_t,float theta_t, float y_a, float theta_a);              //实现模糊控制
    void showInfo();                                      //显示该模糊控制器的信息
    void showMf(const string & type,float *mf_paras);      //显示隶属度函数的信息
};


Fuzzy_controller::Fuzzy_controller(float y_max,float theta_max,float u_max):
y_target(0),y_actual(0),theta_target(0), theta_actual(0), y_max(y_max),theta_max(theta_max),u_max(u_max),y_mf_paras(NULL),theta_mf_paras(NULL),u_mf_paras(NULL)
{}

Fuzzy_controller::~Fuzzy_controller()
{
  delete [] y_mf_paras;
  delete [] theta_mf_paras;
  delete [] u_mf_paras;
}

//三角隶属度函数
float Fuzzy_controller::trimf(float x,float a,float b,float c)
{
   float u;
   if(x>=a&&x<=b)
       u=(x-a)/(b-a);
   else if(x>b&&x<=c)
       u=(c-x)/(c-b);
   else
       u=0.0;
   return u;

}

//梯形隶属度函数
float Fuzzy_controller::L_trapmf(float x,float a,float b,float c)
{
    float u;
    if( (x>=a&&x<b))
        u=1.0;
    else if(x>=b&&x<c)
        u=(c-x)/(c-b);
    else
        u=0.0;
    return u;
}

float Fuzzy_controller::R_trapmf(float x,float a,float b,float c)
{
    float u;
    if(x>=a&&x<=b)
        u=(x-a)/(b-a);
    else if(x>b&&x<c)
        u=1.0;
    else
        u=0.0;
    return u;
}
//设置模糊规则
void Fuzzy_controller::setRule(string rulelist[M][N])
{
    for(int i=0;i<M;i++)
       for(int j=0;j<N;j++)
         rule[i][j]=rulelist[i][j];
}

//设置模糊隶属度函数的类型和参数
void Fuzzy_controller::setMf(float *y_mf,float *theta_mf,float *u_mf)
{
    y_mf_paras=new float [N*3];
    theta_mf_paras=new float [M*3];
    u_mf_paras=new float [M*3];

    for(int i=0;i<N*3;i++)
       y_mf_paras[i]=y_mf[i];
    for(int i=0;i<M*3;i++)
       theta_mf_paras[i]=theta_mf[i];
    for(int i=0;i<M*3;i++)
       u_mf_paras[i]=u_mf[i];
}
//实现模糊控制
float Fuzzy_controller::realize(float y_t,float theta_t, float y_a, float theta_a)   
{
    float u_y[N],u_theta[M],u_u[M]={0.0};
    int u_y_index[3],u_theta_index[3];//假设一个输入最多激活3个模糊子集
    float u;

    y_target=y_t;
    theta_target=theta_t;

    y_actual=y_a;
    theta_actual=theta_a;

    int j=0;
    for(int i=0;i<N;i++)
    {
        //y模糊化，计算它的隶属度
        if(i==0)
        {
            u_y[i] = L_trapmf(y_actual, y_mf_paras[i*3], y_mf_paras[i*3+1], y_mf_paras[i*3+2]);
        }
        else if ( i == M-1)
        {
            u_y[i] = R_trapmf(y_actual, y_mf_paras[i*3], y_mf_paras[i*3+1], y_mf_paras[i*3+2]);
        }
        else
        {
            u_y[i]=trimf(y_actual,y_mf_paras[i*3],y_mf_paras[i*3+1],y_mf_paras[i*3+2]);
        }
        
        if(u_y[i]!=0.0)
            u_y_index[j++]=i;                                              //存储被激活的模糊子集的下标，可以减小计算量
    }
    for(;j<3;j++)u_y_index[j]=-1;

    j=0;
    for(int i=0;i<M;i++)
    {
        //theta模糊化，计算它的隶属度
        if(i==0)
        {
            u_theta[i] = L_trapmf(theta_actual, theta_mf_paras[i*3], theta_mf_paras[i*3+1], theta_mf_paras[i*3+2]);
        }
        else if ( i == M-1)
        {
            u_theta[i] = R_trapmf(theta_actual, theta_mf_paras[i*3], theta_mf_paras[i*3+1], theta_mf_paras[i*3+2]);
        }
        else
        {
            u_theta[i]=trimf(theta_actual,theta_mf_paras[i*3],theta_mf_paras[i*3+1],theta_mf_paras[i*3+2]);
        }
        
        if(u_theta[i]!=0.0)
            u_theta_index[j++]=i;                                              //存储被激活的模糊子集的下标，可以减小计算量
    }
    for(;j<3;j++)u_theta_index[j]=-1;

    for(int i=0; i<3; i++)
    {
        if(u_theta_index[i]!= -1)
        {
            for (int j = 0; j<3; j++)
            {
                if(u_y_index[j]!=-1)
                {
                    if(rule[u_theta_index[i]][u_y_index[j]] == "PB")
                    {
                        u_u[M-1] = min(u_y[u_y_index[j]], u_theta[u_theta_index[i]]);
                    }
                    else if(rule[u_theta_index[i]][u_y_index[j]] == "PM")
                    {
                        u_u[M-2] = min(u_y[u_y_index[j]], u_theta[u_theta_index[i]]);
                    }
                    else if(rule[u_theta_index[i]][u_y_index[j]] == "PS")
                    {
                        u_u[M-3] = min(u_y[u_y_index[j]], u_theta[u_theta_index[i]]);
                    }
                    else if(rule[u_theta_index[i]][u_y_index[j]] == "ZE")
                    {
                        u_u[M-4] = min(u_y[u_y_index[j]], u_theta[u_theta_index[i]]);
                    }
                    else if(rule[u_theta_index[i]][u_y_index[j]] == "NS")
                    {
                        u_u[M-5] = min(u_y[u_y_index[j]], u_theta[u_theta_index[i]]);
                    }
                    else if(rule[u_theta_index[i]][u_y_index[j]] == "NM")
                    {
                        u_u[M-6] = min(u_y[u_y_index[j]], u_theta[u_theta_index[i]]);
                    }
                    else if(rule[u_theta_index[i]][u_y_index[j]] == "NB")
                    {
                        u_u[M-7] = min(u_y[u_y_index[j]], u_theta[u_theta_index[i]]);
                    }
                }
            }
        }
    }
    
    float num = 0.0, den = 0.0;
    for ( float i=-30; i <=30; i=i+0.1)
    {
        if( i>=-30 && i<=-25)
        {
            num += i*min(u_u[0], L_trapmf(i, -30, -30, -17));
            den += min(u_u[0], L_trapmf(i, -30, -30, -17));
        }

        if( i>=-25 && i<=-17)
        {
            num += i*max(min(u_u[0], L_trapmf(i, -30, -30, -17)),min(u_u[1], trimf(i,-25,-15,-5)));
            den += max(min(u_u[0], L_trapmf(i, -30, -30, -17)),min(u_u[1], trimf(i,-25,-15,-5)));
        }
        if( i>=-17 && i<=-12.5)
        {
            num += i*min(u_u[1], trimf(i,-25,-15,-5));
            den += min(u_u[1], trimf(i,-25,-15,-5));
        }

        if( i>=-12.5 && i<=-5)
        {
            num += i*max(min(u_u[1], trimf(i,-25,-15,-5)),min(u_u[2], trimf(i,-12.5,-7,0)));
            den += max(min(u_u[1], trimf(i,-25,-15,-5)),min(u_u[2], trimf(i,-12.5,-7,0)));
        }

        if( i>=-5 && i<=0)
        {
            num += i*max(min(u_u[2], trimf(i,-12.5,-7,0)),min(u_u[3], trimf(i,-5,0,5)));
            den += max(min(u_u[2], trimf(i,-12.5,-7,0)),min(u_u[3], trimf(i,-5,0,5)));
        }

        if( i>=0 && i<=5)
        {
            num += i*max(min(u_u[3], trimf(i,-5,0,5)),min(u_u[4], trimf(i,0,7,12.5)));
            den += max(min(u_u[3], trimf(i,-5,0,5)),min(u_u[4], trimf(i,0,7,12.5)));
        }
        if( i>=5 && i<=12.5)
        {
            num += i*max(min(u_u[4], trimf(i,0,7,12.5)),min(u_u[5], trimf(i,5,15,25)));
            den += max(min(u_u[4], trimf(i,0,7,12.5)),min(u_u[5], trimf(i,5,15,25)));
        }

        if( i>=12.5 && i<=17)
        {
            num += i*min(u_u[5], trimf(i,5,15,25));
            den += min(u_u[5], trimf(i,5,15,25));
        }

        if( i>=17 && i<=25)
        {
            num += i*max(min(u_u[6], R_trapmf(i, 17,30,30)),min(u_u[5], trimf(i,5,15,25)));
            den += max(min(u_u[6], R_trapmf(i, 17,30,30)),min(u_u[5], trimf(i,5,15,25)));
        }
        if( i>=25 && i<=30)
        {
            num += i*min(u_u[6], R_trapmf(i, 17,30,30));
            den += min(u_u[6], R_trapmf(i, 17,30,30));
        }
    }
    u=num/den;
    if(u>=u_max || y_actual<=-y_max || theta_actual<=-theta_max)   
    {
        u=u_max;
    }
    else if(u<=-u_max|| y_actual>=y_max || theta_actual>=theta_max)  
    {
        u=-u_max;
    }
    return u;
}
// void Fuzzy_controller::showMf(const string & type,float *mf_paras)
// {
//     int tab;
//     if(type=="trimf")
//         tab=2;
//     else if(type=="gaussmf")
//         tab==1;
//     else if(type=="trapmf")
//         tab=3;
//     cout<<"函数类型："<<mf_t_y<<endl;
//     cout<<"函数参数列表："<<endl;
//     float *p=mf_paras;
//     for(int i=0;i<N*(tab+1);i++)
//       {
//           cout.width(3);
//           cout<<p[i]<<"  ";
//           if(i%3==tab)
//               cout<<endl;
//       }
// }
void Fuzzy_controller::showInfo()
{
   cout<<"Info of this fuzzy controller is as following:"<<endl;
   cout<<"基本论域y：["<<-y_max<<","<<y_max<<"]"<<endl;
   cout<<"基本论域theta：["<<-theta_max<<","<<theta_max<<"]"<<endl;
   cout<<"基本论域u：["<<-u_max<<","<<u_max<<"]"<<endl;

   cout<<"模糊规则表："<<endl;
   for(int i=0;i<N;i++)
   {
     for(int j=0;j<N;j++)
       {
         cout.width(3);
         cout<<rule[i][j]<<"  ";
        }
       cout<<endl;
   }
}