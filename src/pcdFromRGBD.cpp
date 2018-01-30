#include <vector>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <typeinfo>

#include "loadParameters.h"
#include "generatePointCloud.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp> // if use cvtColor

using namespace std;
using namespace cv;

Mat readFrame( int index, string imagePath, string groupName, string Ext );
Mat readMatrix( string MaskString );

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

    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx       = atof( pd.getData( "Camera.fx" ).c_str());
    camera.fy       = atof( pd.getData( "Camera.fy" ).c_str());
    camera.cx       = atof( pd.getData( "Camera.cx" ).c_str());
    camera.cy       = atof( pd.getData( "Camera.cy" ).c_str());
    camera.scale    = atof( pd.getData( "DepthMapFactor" ).c_str() );


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

            FRAME framePCD;

            string ssRGB = image_Path+sRGB;
            // cout << ssRGB <<endl;
            framePCD.rgb = cv::imread( ssRGB, CV_LOAD_IMAGE_UNCHANGED);

            string ssDepth = image_Path+sD;
            // cout << ssDepth <<endl;
            framePCD.depth = cv::imread( ssDepth, CV_LOAD_IMAGE_UNCHANGED);

            cv::Mat imD = cv::imread( ssDepth,  IMREAD_UNCHANGED);
            for(int row = 0; row < imD.rows; row++)

            {
                for(int col = 0; col < imD.cols; col++)
                {

                     cout << imD.at<float>(row, col) << endl;
                }
            }

            PointCloud::Ptr cloud = image2PointCloud( framePCD.rgb, framePCD.depth, camera );

            string cloud_path;
            cloud_path = output_Path + sTimestamp + ".pcd";
            cout << cloud_path <<endl;
            pcl::io::savePCDFile(cloud_path, *cloud);
            cloud->points.clear();
        }
    }

    return 0;
}
