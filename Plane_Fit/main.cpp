#include <iostream>
#include<stdio.h>
#include<math.h>
#include<Eigen/Dense>
#include<utility>

using namespace std;
using namespace Eigen;

struct plane {
    Vector3d Normals;
    double inlier_ratio;
    double points_avg;
    double  pos;
};


double calculate_distance(Vector3d normal,Vector3d points,double D)
{
    double x1 = points(0);
    double y1 = points(1);
    double z1 = points(2);
    double A = normal(0);
    double B = normal(1);
    double C = normal(2);
    double dist;


    dist = abs(A*x1 + B*y1 + C*z1 + D)/(sqrt(pow(A,2) + pow(B,2) + pow(C,2)));
    return dist;
}

 plane plain_fitting(double,double,vector<Vector3d>&points);

int main()
{
    Vector3d plane_normals;
    //parameters
    double ransac_thresh = 1.5;
    double ransac_ratio = 0.6; //ratio of inliers in the sample data
    const int n_samples = 100;

    //plain fitting function declaration


    //generate the dataset
    srand(time(NULL));

    double m = 0.05;
    float random = 0.5f;
    float noise = 0.f;

    const static int q = 15;
    const static float c1 = (1 << q) - 1;
    const static float c2 = ((int)(c1 / 3)) + 1;
    const static float c3 = 1.f / c1;

    random = ((float)rand()/(float)RAND_MAX + 1);
    noise = (2.f * ((random * c2) + (random * c2) + (random * c2)) - 3.f * (c2 - 1.f)) * c3;
    float noise_scale = 2.0;

    vector<Vector3d>points;
    Vector3d ind;
    Vector3d Noise(noise*noise_scale,noise*noise_scale,noise*noise_scale);

    for(int i = 0;i<n_samples;i++)
    {

        int j=0;
        while(j<3)
        {
            if(j ==0)
                ind(0) = i;
            if(j == 1)
                ind(1) = pow(ind(0)+1,2);
            if(j == 2)
                ind(2) = 2*(ind(1) + 1);
            j++;
        }
        points.push_back(ind);
    }
    for(int j=0;j<points.size();j++)
    {
        if(j > 40 && j<50)
            points.at(j) = points.at(j) + Noise;
        cout<<points.at(j).transpose()<<endl;

    }

                                 /* you can use your own dataset */
//vector<Vector3d>points;
//Vector3d p1 = Vector3d(1,3,6);
//Vector3d p2 = Vector3d(1,5,6);
//Vector3d p3 = Vector3d(2,1,6);
//Vector3d p4 = Vector3d(13,3,0);
//Vector3d p5 = Vector3d(2,1,7);
//points.push_back(p1);
//points.push_back(p2);
//points.push_back(p3);
//points.push_back(p4);
//points.push_back(p5);

    plane new_val = plain_fitting(ransac_ratio,ransac_thresh,points);
    plane_normals = new_val.Normals;
    cout<<"The avg distance of the points to the plane is: "<<new_val.points_avg<<endl;
    cout<<"The inlier ratio is: "<<new_val.inlier_ratio<<endl;
    cout<<"The normals are: "<<plane_normals(0)<<" "<<plane_normals(1)<<" "<<plane_normals(2)<<endl;
    cout<<"D is: "<<new_val.pos<<endl;


    return(0);

}








plane plain_fitting(double ransac_ratio,double ransac_thresh,vector<Vector3d>&points)
{
    plane v;
    int n_samples = points.size();

    random_device rd;
    mt19937 g(rd());
    float ratio = 0.0;
    vector<int>indices;
    for(int i =0;i<n_samples;i++)
        indices.push_back(i);
    double p1,p2;


    int s= 3;
    //ransac iterations
    double pro = 0.99;  //probability that at least 1 subset of the data does not contain any outliers
    double ransac_iter = log(1 -pro)/log(1 - pow(ransac_ratio,3));
    cout<<"iter: "<<ransac_iter<<endl;
    for(int it=0;it<ransac_iter;it++)
    {
        cout<<"iter = "<<it<<endl;
        int p =0;
        Vector3d A,B,C, AB,BC,normal;
        vector<vector<double>>maybe_points;
        vector<vector<double>>test_points;
        shuffle(indices.begin(),indices.end(),g);
        for(int j =0;j<indices.size();j++)
        {

            vector<double>maybe;
            vector<double>test;
            if(j<s)
            {
                int idx1 = indices.at(j);
                //cout<<"idx: "<<idx1<<endl;
                while(p<3)
                {
                    // p1 = data[idx1][p];
                    p1 = (points.at(idx1))(p);
                    // cout<<"p1: "<<p1<<" ";
                    maybe.push_back(p1);

                    p++;
                }
                maybe_points.push_back(maybe);
            }
            p = 0;
            if(j>=s)
            {
                int idx2 = indices.at(j);
                while(p<3)
                {
                    // p2 = data[idx2][p];
                    p2 = (points.at(idx2))(p);
                    test.push_back(p2);
                    p++;
                }
                test_points.push_back(test);
            }


        }
        for(int i=0;i<3;i++)
        {

            if(i == 0)
            {
                //                  cout<<maybe_points.at(i).at(0)<<"A(0)"<<endl;
                //                  cout<<maybe_points.at(i).at(1)<<"A(1)"<<endl;
                //                  cout<<maybe_points.at(i).at(2)<<"A(2)"<<endl;
                A(0) = maybe_points.at(i).at(0);
                A(1) = maybe_points.at(i).at(1);
                A(2) = maybe_points.at(i).at(2);
            }
            if(i ==1)
            {
                B(0) = maybe_points.at(i).at(0);
                B(1) = maybe_points.at(i).at(1);
                B(2) = maybe_points.at(i).at(2);
            }
            if(i == 2)
            {
                C(0) = maybe_points.at(i).at(0);
                C(1) = maybe_points.at(i).at(1);
                C(2) = maybe_points.at(i).at(2);
            }

        }
        AB = B-A;
        BC = C-B;


        normal = AB.cross(BC);
        normal.normalize();

        double d = -normal.dot(A);


        vector<double>x_outliers;
        vector<double>y_outliers;
        vector<double>z_outliers;
        vector<double>points_dist;
        vector<double>out_dist;

        Vector3d test_pts;
        int num = 0;
        double dist;
        double Sum = 0;
        for(int id=0;id<test_points.size();id++)
        {
            double x0 = test_points.at(id).at(0);
            double y0 = test_points.at(id).at(1);
            double z0 = test_points.at(id).at(2);
            // cout<<"x:"<<x0<<"y:"<<y0<<"z:"<<z0<<endl;
            test_pts(0) = x0;
            test_pts(1) = y0;
            test_pts(2) = z0;
            dist = calculate_distance(normal,test_pts,d);
            points_dist.push_back(dist);
             cout<<"distance: "<<dist<<endl;
            if(dist < ransac_thresh)
            {
                //                x_inliers.push_back(x0);
                //                y_inliers.push_back(y0);
                //                z_inliers.push_back(z0);
                num++;
                //cout<<"num: "<<num<<endl;


            }

            if(dist > ransac_thresh)
            {
                out_dist.push_back(dist);
                x_outliers.push_back(x0);
                y_outliers.push_back(y0);
                z_outliers.push_back(z0);
            }

        }
        for(int j=0;j<points_dist.size();j++)
        {
            Sum += points_dist.at(j);

        }
        double avg = Sum/n_samples;
        cout<<"avg is: "<<avg<<endl;


                for(int i=0;i<x_outliers.size();i++)
                {
                    cout<<"outliers: "<<x_outliers.at(i)<<" "<<y_outliers.at(i)<<" "<<z_outliers.at(i)<<endl;
                    cout<<"outliers distance: "<<out_dist.at(i)<<endl<<endl;
                }
        


        num = num+3;
        // cout<<"num: "<<num<<endl;
        if(num/(float)n_samples > ratio)
        {
            ratio = num/(float)n_samples;
            v.inlier_ratio = ratio;
            v.Normals = normal;
            v.pos = d;
            v.points_avg = avg;

        }
        cout<<"Inlier ratio: "<<ratio<<endl;



        if(num > n_samples*ransac_ratio)
        {


            cout<<"The model is found"<<endl;
            break;


        }
        else if(num < n_samples*ransac_ratio && it == (int)ransac_iter)
        {
            cout<<"Model not found!!!"<<endl;
        }
    }
    return v;
}





