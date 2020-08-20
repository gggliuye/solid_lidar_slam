#ifndef POINT_CLOUD_PROJECTOR_H
#define POINT_CLOUD_PROJECTOR_H

#include "LidarUtils.h"

#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h> 

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>

struct UVandZ
{
    int u;
    int v;
    float z;
};

class VirtualCamera
{
public:    
    VirtualCamera();
    ~VirtualCamera(){}
  
    void SetCameraIntrinsic(float fx_, float fy_, float cx_, float cy_, int rows_, int cols_);

    // result is (u, v, z), it is not any vector3 in any reference frame
    UVandZ ProjectPointToImage(float x, float y, float z);

    bool PointInRange(UVandZ vuz);

    bool ProjectPointCloudToImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat &image, cv::Mat &depth);
    bool ProjectPointCloudToExistImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat &image, cv::Mat &depth);

private:
    bool bCameraSet;
    float fx;
    float fy;
    float cx;
    float cy;
    int rows, cols;

    // range image PCL and visualization
public:
    std::mutex m_b_updated;
    bool bCloudUpdated = false;

    float offset_x = 0.0;
    float offset_y = -0.045; // height offset
    float offset_z = -0.095;

    std::mutex m_range_image;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;

    void ProjectionPclRangeImage();
    void SetPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in);

};

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
namespace NormalEstimation
{
    
    void TestNormalEstimation(std::string filename);
    void TestNormalEstimationOrganized(std::string filename);

}

namespace FAILED
{

    void TestRangeImageSimple();
    void TestOffical(std::string filename);

}

#endif
