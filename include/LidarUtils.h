#ifndef LIDAR_UTILS_H
#define LIDAR_UTILS_H

#include <iostream>
#include <vector>
#include <mutex>
#include <stdlib.h>
#include <stdio.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h> 
#include <pcl/filters/voxel_grid.h>

#define rand_between(a, b) ((rand() % (b-a+1))+ a)

#define VERBOSE 1

namespace GUtils{

void RandomDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float rate);

void RandomDownSampleRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, float rate);

void RandomDownSampleAndFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float rate);

bool LoadPcdPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName);

bool LoadPcdRGBAPoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string fileName);

bool LoadPlyRGBPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string fileName);

void CombinePointAndNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal,
                           pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal);

void CombinePointAndNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal,
                           pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal);

bool CheckExistance(std::vector<int> &intVector, const int &numberToCheck);
}
#endif
