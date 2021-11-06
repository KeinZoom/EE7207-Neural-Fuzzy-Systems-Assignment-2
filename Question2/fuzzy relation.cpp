#include <iostream>
#include <vector>
#include <math.h>
#include "algorithm"
#include "iterator"
using namespace std;
#define M 16
#define N 4

vector<vector<float>> data_set = 
{
    {0.1, 0.7, 0.2, 0.0},
    {0.0, 0.5, 0.5, 0.0},
    {0.2, 0.2, 0.2, 0.4},
    {0.8, 0.1, 0.0, 0.1},
    {0.3, 0.0, 0.4, 0.3},
    {0.0, 0.4, 0.0, 0.6},
    {0.5, 0.0, 0.4, 0.1},
    {0.6, 0.3, 0.0, 0.1},
    {0.0, 0.5, 0.1, 0.4},
    {0.1, 0.6, 0.0, 0.3},
    {0.3, 0.2, 0.1, 0.4},
    {0.1, 0.5, 0.4, 0.0},
    {0.2, 0.0, 0.2, 0.6},
    {0.2, 0.6, 0.1, 0.1},
    {0.1, 0.7, 0.1, 0.1},
    {0.2, 0.4, 0.2, 0.2}
};

vector<vector<float>> R1(M);


void gen_cosampti(vector<vector<float>> &R1, vector<vector<float>> &data)
{
    cout << " fuzzy tolerance relation R1 "<<endl;
    for(int i = 0; i < M; i++)
    {
        for ( int j = 0; j<M; j++)
        {
            float num=0.0, den=0.0, den1=0.0,den2=0.0;
            for ( int k = 0; k < N ; k++)
            {
                num += data[i][k] * data[j][k];
                den1 += data[i][k] * data[i][k];
                den2 += data[j][k] * data[j][k];
            }

            num = abs(num);
            den = sqrt (den1 * den2);
            // R1[i].push_back(num / den);
            R1[i].push_back(floor(num / den*100)/100);
            cout << R1[i][j] << " " ;
        }
        cout << endl;
    }
}

vector<float> vectors_min(vector<float> v1,vector<float> v2){
    vector<float> v(v1.size());
    for (int i=0; i<v1.size(); i++)
    {
        v[i] = min(v1[i], v2[i]);
    }   
    return v;
}

void compo_ope(vector<vector<float>> &temp_R)
{

    for (int i = 0; i<M; i++)
    {
        for ( int j = 0; j<M; j++)
        {
            vector<float> v = vectors_min(temp_R[i],temp_R[j]);
            temp_R[i][j] = *max_element(v.begin(), v.end());
        }
    }
}

void gen_R(vector<vector<float>> &R)
{

    vector<vector<float>> temp_R = R;
    compo_ope(temp_R);
    while(temp_R != R)
    {
        R = temp_R;
        compo_ope(temp_R);
    }
    cout << " fuzzy equivalence relation R " << endl;
    for(int i = 0; i < M; i++)
    {
        for ( int j = 0; j<M; j++)
        {
            cout << temp_R[i][j] << " " ;
        }
        cout << endl;
    }
}

void gen_alpha_cut(vector<vector<float>> R, float alpha)
{
    for(int i = 0; i < M; i++)
    {
        for ( int j = 0; j<M; j++)
        {
            if( R[i][j] >= alpha)
                cout << 1 << " " ;
            else
                cout << 0 << " " ;
        }
        cout << endl;
    }
}

int main()
{
    gen_cosampti(R1, data_set);
    vector<vector<float>> R = R1;
    gen_R(R);
    cout << " alpha cut is 0.4" << endl;
    gen_alpha_cut(R, 0.4);
    cout << " alpha cut is 0.8" << endl;
    gen_alpha_cut(R, 0.8);
    cout << " alpha cut is 0.85" << endl;
    gen_alpha_cut(R, 0.85);
    return 0;
}