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
#include <pcl/point_cloud.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;


struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};


void generatePointCloud(CAMERA_INTRINSIC_PARAMETERS camera, cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, int skip ) 
{  
    double z; 
    double px, py, pz; 
    int height = rgb.rows/skip; 
    int width = rgb.cols/skip; 
    int N = (rgb.rows/skip)*(rgb.cols/skip); 


    pc->points.reserve(N); 
    // pc.width = width; 
    // pc.height = height; 

    unsigned char r, g, b; 
    int pixel_data_size = 3; 
    if(rgb.type() == CV_8UC1)
    {
    pixel_data_size = 1; 
    }

    int color_idx; 
    char red_idx = 2, green_idx =1, blue_idx = 0;

    PointT pt; 

    for(int v = 0; v<rgb.rows; v+=skip)
    for(int u = 0; u<rgb.cols; u+=skip)
    {
    // Point& pt = pc.points[v*width + u]; 
    z = dpt.at<float>((v), (u));
    if(std::isnan(z) || z <= 0.1 || z >= 5) 
    {
      continue; 
    }

    // compute point 
    pz = z/camera.scale; 
    px = ((u-camera.cx)/camera.fx)*pz;
    py = ((v-camera.cy)/camera.fy)*pz;

    pt.x = px;  pt.y = py;  pt.z = pz; 
    color_idx = (v*rgb.cols + u)*pixel_data_size;
    if(pixel_data_size == 3)
    {
      r = rgb.at<uint8_t>(color_idx + red_idx);
      g = rgb.at<uint8_t>(color_idx + green_idx); 
      b = rgb.at<uint8_t>(color_idx + blue_idx);
    }else{
      r = g = b = rgb.at<uint8_t>(color_idx); 
    }
    pt.r = r; pt.g = g; pt.b = b; 
    pc->points.push_back(pt); 
    }
    pc->height = 1; 
    pc->width = pc->points.size(); 
    return ;
}


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

            string ssRGB = image_Path+sRGB;
            // cout << ssRGB <<endl;

            string ssDepth = image_Path+sD;
            cout << ssDepth <<endl;


            cv::Mat rgb, depth;
            rgb = cv::imread( ssRGB,  CV_LOAD_IMAGE_UNCHANGED);
            depth = cv::imread( ssDepth,  CV_LOAD_IMAGE_UNCHANGED);

            PointCloud::Ptr cloud ( new PointCloud );

            generatePointCloud(camera,rgb, depth, cloud, 1) ;

            string cloud_path;
            cloud_path = output_Path + sTimestamp + ".pcd";
            cout << cloud_path <<endl;
            pcl::io::savePCDFile(cloud_path, *cloud);
            cloud->points.clear();
        }
    }

    return 0;
}
