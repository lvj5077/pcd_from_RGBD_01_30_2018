#include <vector>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <typeinfo>

#include "loadParameters.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp> // if use cvtColor

using namespace std;
using namespace cv;


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


int main(int argc, char** argv)
{
    cout<<endl<<"Program Started!"<<endl;
    cout <<"~~~~~~~~~~~~~~~~~~"<<endl<<endl;

    string ParameterPath = "/home/jin/Desktop/pcd_from_RGBD_01_30_2018/parameters.txt";
    if(argc >=2){
    	ParameterPath = argv[1];
    }

    ParameterReader pd(ParameterPath);
    string image_Path       = pd.getData( "image_Path" );
    string timestamp_Path   = pd.getData( "timestamp_Path" );
    string output_Path      = pd.getData( "output_Path" );

    double camera_fx       = atof( pd.getData( "Camera.fx" ).c_str());
    double camera_fy       = atof( pd.getData( "Camera.fy" ).c_str());
    double camera_cx       = atof( pd.getData( "Camera.cx" ).c_str());
    double camera_cy       = atof( pd.getData( "Camera.cy" ).c_str());
    double camera_scale    = atof( pd.getData( "DepthMapFactor" ).c_str() );


    ifstream fAssociation(timestamp_Path.c_str() );
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t_d;
            string sRGB, sD, sTimestamp;

            ss >> sTimestamp;
            ss >> sRGB;
            ss >> t_d;
            ss >> sD;

            string ssRGB = image_Path+sRGB;
            // cout << ssRGB <<endl;

            string ssDepth = image_Path+sD;
            cout << ssDepth <<endl;


            cv::Mat rgb, depth;
            rgb = cv::imread( ssRGB,  CV_LOAD_IMAGE_UNCHANGED);
            depth = cv::imread( ssDepth,  CV_LOAD_IMAGE_UNCHANGED);

            PointCloud::Ptr cloud ( new PointCloud );

            for (int m = 0; m < depth.rows; m++)
                for (int n=0; n < depth.cols; n++)
                {
                    ushort d = depth.ptr<ushort>(m)[n];
                    if (d == 0)
                        continue;
                    PointT p;

                    p.z = double(d) / camera_scale;
                    p.x = (n - camera_cx) * p.z / camera_fx ;
                    p.y = (m - camera_cy) * p.z / camera_fy;
                    
 
                    p.b = rgb.ptr<uchar>(m)[n*3];
                    p.g = rgb.ptr<uchar>(m)[n*3+1];
                    p.r = rgb.ptr<uchar>(m)[n*3+2];

                    cloud->points.push_back( p );
                }

            cloud->height = 1;
            cloud->width = cloud->points.size();
            cout<<"point cloud size = "<<cloud->points.size()<<endl;
            cloud->is_dense = false;

            string cloud_path;
            cloud_path = output_Path + sTimestamp + ".pcd";
            cout << cloud_path <<endl;
            pcl::io::savePCDFile(cloud_path, *cloud);
            cloud->points.clear();
        }
    }

    return 0;
}
