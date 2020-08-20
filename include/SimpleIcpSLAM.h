#ifndef SIMPLE_ICP_SLAM
#define SIMPLE_ICP_SLAM

#include "LidarUtils.h"
#include <pcl/registration/ndt.h>
#include "pclomp/ndt_omp.h"
#include <pclomp/gicp_omp.h>

#include <pcl/octree/octree.h>

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>

#define USERGBA 1;

using namespace std;

using namespace GUtils;

class GLidarSLAM{

public:
    GLidarSLAM();
    ~GLidarSLAM(){}

    void ReSet();

    void AddFrameFile(string fileName);

    void AddFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new);

    bool MatchSimpleICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_old, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int iterations);

    bool MatchPlaneICP(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int iterations);

    bool MatchNdtOmp(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_old, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int resolution, int iterations);

    bool MatchNdt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_old, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int resolution, int iterations);

    bool GetPoses(std::vector<Eigen::Matrix4d> &output);

    void SetFirstFrameWithNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_first);

private:
    bool firstFlag = true;
    bool use_normal = false;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_old;
    pcl::PointCloud<pcl::PointNormal> cloud_normal_old;

    float downSampleRate = 0.4;

    // initial gauss of transformation matrix
    Eigen::Matrix4d transformation_matrix_initial;

    // ICP iteration parameters
    int icpMaxIterations = 20;
    double maxCorrespondenceDistance = 0.2;
    double ransacThreshold = 0.9;

    // NDT iteration parameters
    double ndtResolution = 8;
    int ndtIterations = 100;

public:
    std::mutex m_map;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map_ptr;

    std::mutex m_changedPoint;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_changedPoint;

private:
    bool bTestChangedPoints = false;
    float fCarHeight = 1.0;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_changed;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_changed_ptr;

    void ConstumeRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seed,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grown);

    int CompareAndFindOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_old, 
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                              Eigen::Matrix4d transformation_matrix,
                              Eigen::Matrix4d transformation_matrix_to_world);
private:
    int frameCount = 0;
    std::vector<std::string> vPcdFilePaths;                 // unused
    std::vector<Eigen::Matrix4d> vRelativeTransformations;
    std::mutex m_pose;
    std::vector<Eigen::Matrix4d> vSensorTransformations;

};

#endif




