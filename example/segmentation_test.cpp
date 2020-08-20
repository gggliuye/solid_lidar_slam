#include <iostream>
#include <thread>

#include "GSegmentation.h"
#include "viewer.h"

using namespace std;

std::shared_ptr<GSegmentation> gSegmentation;

string FilePath = "/home/viki/Documents/LidarSLAM/Neuvition/neuvition_shenlan_rgbpcd/";

void ReceiveLidarData()
{
    int i = 1;
    while (i < 1951) {
        cout << "FRAME " << i << endl;
        string fileName = FilePath + to_string(i) + ".pcd";

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGBA>);
        GUtils::LoadPcdRGBAPoints(cloud_color, fileName);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        GUtils::LoadPcdPoints(cloud, fileName);

        if(!gSegmentation->SegmentationPointCloud(cloud))
            continue;

        gSegmentation->ExtractInliersInTheCloud(cloud);
        gSegmentation->EraseInliersInTheCloud(cloud_color);

        gSegmentation->m_show_clouds.lock();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
        (*cloud_temp) += (*cloud);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
        (*cloud_color_temp) += (*cloud_color);
        gSegmentation->cloud_plane = cloud_temp;
        gSegmentation->cloud_other = cloud_color_temp;
        gSegmentation->m_show_clouds.unlock();

        gSegmentation->m_state.lock();
        gSegmentation->segmented = true;
        gSegmentation->m_state.unlock();

        i = i + 1;
    }
}

void Viewer()
{
    GLidarSegmentationViewer(gSegmentation);
}

int main(int argc, char** argv)
{
    cout << " [GLSLAM] lidar slam using Neuvition solid state lidar device. \n";
    cout << "  build by Ye LIU, UTOPA, GuangZhou. \n\n";

    gSegmentation.reset(new GSegmentation());

    std::thread thd_ReceiveLidar(ReceiveLidarData);
    std::thread thd_Viewer(Viewer);

    thd_ReceiveLidar.join();
    thd_Viewer.join();

    return 0;
}

