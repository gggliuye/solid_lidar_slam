#include "GSegmentation.h"


void GSegmentation::EraseInliersInTheCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    if (inliers.indices.size () == 0){
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator index = cloud->begin();
    // inliers' indices is ranked from small to big as default
    // if we erase a point, the indices should also be substracted by 1
    for (std::size_t i = 0; i < inliers.indices.size (); ++i){
        index = cloud->begin() + inliers.indices[i] - i;
        cloud->erase(index);
    }
}

void GSegmentation::ExtractInliersInTheCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (inliers.indices.size () == 0){
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    (*cloud_temp) += (*cloud);
    cloud->clear();

    for (std::size_t i = 0; i < inliers.indices.size(); ++i){
        int index = inliers.indices[i];
        pcl::PointXYZ pt = cloud_temp->points[index];
        cloud->points.push_back(pt);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}

bool GSegmentation::SegmentationPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
//    pcl::ModelCoefficients::Ptr pCoefficients = coefficients.makeShared();
//    pcl::PointIndices::Ptr pInliers = inliers.makeShared();

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);

    seg.setInputCloud (cloud);
    seg.segment (inliers, coefficients);
    if (inliers.indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset. \n");
        return false;
    }

    std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
                                        << coefficients.values[1] << " "
                                        << coefficients.values[2] << " " 
                                        << coefficients.values[3] << std::endl;
/*
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (std::size_t i = 0; i < inliers->indices.size (); ++i)
        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                   << cloud->points[inliers->indices[i]].y << " "
                                                   << cloud->points[inliers->indices[i]].z << std::endl;
*/
    return true;
}
