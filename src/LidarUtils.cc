#include "LidarUtils.h"

namespace GUtils{

void RandomDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float rate)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    (*cloud_temp) += (*cloud);
    cloud->clear();
    for(size_t i = 0; i < cloud_temp->points.size();i++)
    {
       float r = float(rand_between(0, 100)) / 100.0;
       if(r < rate)
       {
         pcl::PointXYZ pt = cloud_temp->points[i];
         cloud->points.push_back(pt);
       }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}

void RandomDownSampleRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, float rate)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
    (*cloud_temp) += (*cloud);
    cloud->clear();
    for(size_t i = 0; i < cloud_temp->points.size();i++)
    {
       float r = float(rand_between(0, 100)) / 100.0;
       if(r < rate)
       {
         pcl::PointXYZRGBA pt = cloud_temp->points[i];
         cloud->points.push_back(pt);
       }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}

void RandomDownSampleAndFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float rate)
{
    float distance_threshold_min = 2.0;
    float distance_threshold_max = 15.0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    (*cloud_temp) += (*cloud);
    cloud->clear();
    for(size_t i = 0; i < cloud_temp->points.size();i++)
    {
       float r = float(rand_between(0, 100)) / 100.0;
       if(r < rate)
       {
           pcl::PointXYZRGB pt = cloud_temp->points[i];
           if(pt.z > distance_threshold_max || pt.z < distance_threshold_min){
               continue;
           }
           cloud->points.push_back(pt);
       }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}


bool LoadPcdPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName)
{ 
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file. \n"); 
        return false;
    }

    if(VERBOSE){
        std::cout << "Loaded "
                  << cloud->width << " * " << cloud->height
                  << " data points from " << fileName << std::endl;
    }

    return true;
}

bool LoadPcdRGBAPoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string fileName)
{ 
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (fileName, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file. \n"); 
        return false;
    }

    if(VERBOSE){
        std::cout << "Loaded "
                  << cloud->width << " * " << cloud->height
                  << " data points from " << fileName << std::endl;
    }

    return true;
}

bool LoadPlyRGBPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string fileName)
{ 
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (fileName, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file. \n"); 
        return false;
    }

    if(VERBOSE){
        std::cout << "Loaded "
                  << cloud->width << " * " << cloud->height
                  << " data points from " << fileName << std::endl;
    }

    return true;
}

void CombinePointAndNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal,
                           pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal)
{
    pointNormal->clear();
    if(cloud->points.size() != normal->points.size()){
        PCL_ERROR ("Points and normals should have the same size. \n"); 
        return;
    }

    for(size_t i = 0; i < cloud->points.size();i++){
       pcl::PointNormal pt;
       pt.x = cloud->points[i].x;
       pt.y = cloud->points[i].y;
       pt.z = cloud->points[i].z;
       pt.normal_x = normal->points[i].normal_x;
       pt.normal_y = normal->points[i].normal_y;
       pt.normal_z = normal->points[i].normal_z;
       pointNormal->points.push_back(pt);
    }
    pointNormal->width = cloud->points.size();
    pointNormal->height = 1;
    pointNormal->is_dense = false;
}

void CombinePointAndNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal,
                           pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal)
{
    pointNormal->clear();
    if(cloud->points.size() != normal->points.size()){
        PCL_ERROR ("Points and normals should have the same size. \n"); 
        return;
    }

    for(size_t i = 0; i < cloud->points.size();i++){
       pcl::PointNormal pt;
       pt.x = cloud->points[i].x;
       pt.y = cloud->points[i].y;
       pt.z = cloud->points[i].z;
       pt.normal_x = normal->points[i].normal_x;
       pt.normal_y = normal->points[i].normal_y;
       pt.normal_z = normal->points[i].normal_z;
       pointNormal->points.push_back(pt);
    }
    pointNormal->width = cloud->points.size();
    pointNormal->height = 1;
    pointNormal->is_dense = false;
}


bool CheckExistance(std::vector<int> &intVector, const int &numberToCheck)
{
    for(size_t i = 0; i < intVector.size(); i++){
        if(intVector[i] == numberToCheck)
            return true;
    }
    return false;
}

} // namespace
