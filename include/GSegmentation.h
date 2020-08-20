#ifndef G_SEGMENTATION_H
#define G_SEGMENTATION_H

#include "LidarUtils.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//using namespace GUtils;

class GSegmentation
{

public:
    GSegmentation(){}
    ~GSegmentation(){}

public:
    bool SegmentationPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void EraseInliersInTheCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

    void ExtractInliersInTheCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

public:
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;

public:
    std::mutex m_state;
    bool segmented = false;

    std::mutex m_show_clouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_other;
};




#endif // #ifndef G_SEGMENTATION_H
