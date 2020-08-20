#include <iostream>
#include <thread>

#include "PointCloudProjector.h"

using namespace std;

//string FilePath = "/home/viki/Documents/LidarSLAM/Neuvition/neuvition_shenlan_rgbpcd/";

string FilePath = "/home/viki/UTOPA/Neuvition/Data_calibration/point_clouds/";
string image_FilePath = "/home/viki/UTOPA/Neuvition/Data_calibration/images/";

static bool compare_timestamp(const int64_t &r1, const int64_t &r2) {
    return r1 < r2;
}

std::vector<int64_t> ReadTimestamps(std::string image_file)
{
    std::vector<int64_t> vTimestamps;
    {
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (image_file.c_str())) == NULL) {
            std::cerr << "[ERROR] cannot open images folder ! " << std::endl;
            return vTimestamps;
        }
        while ((ent = readdir (dir)) != NULL) {
            std::string pathimg = image_file + ent->d_name;
            std::string name_t = ent->d_name;
            std::string file_name = name_t.substr(0, 16);

            std::stringstream a;
            a << file_name;
            int64_t time_stamp = 0;
            a >> time_stamp;

            //int time_stamp = atoi(file_name.c_str());
            if(pathimg.length() - image_file.length() < 4){
                continue;
            }

            vTimestamps.push_back(time_stamp);
            //std::cout << file_name << " " << time_stamp << " " << pathimg << std::endl;
        }
    }
    std::sort(vTimestamps.begin(), vTimestamps.end(), compare_timestamp);

    return vTimestamps;
}

void ConstumeProjector(int argc, char** argv)
{
    float focus_length = 800.0;
     
    if(argc == 2){
        focus_length = atof(argv[1]);
        cout << " focus length set to " << focus_length << endl;
    }

    float cx2 = 1280;
    float cy2 = 720;

    VirtualCamera virtualCamera;
    virtualCamera.SetCameraIntrinsic(focus_length,focus_length,cx2/2.0,cy2/2.0,cx2,cy2);

    std::vector<int64_t> vTimestamps_cloud = ReadTimestamps(FilePath);
    std::vector<int64_t> vTimestamps_image = ReadTimestamps(image_FilePath);

    size_t image_id = 0;
    for(size_t i = 0 ; i < vTimestamps_cloud.size() ; i ++){
        std::cout << "\nProcess cloud with time stamp : " << vTimestamps_cloud[i] << "\n" ;
        std::string file_cloud = FilePath + std::to_string(vTimestamps_cloud[i]) + ".ply";

        while(vTimestamps_image[image_id] < vTimestamps_cloud[i]){
            image_id++;
        }
        if(image_id > vTimestamps_image.size() - 1){
            break;
        }
        std::string file_image = image_FilePath + std::to_string(vTimestamps_image[image_id-1]) + ".jpg";
        cv::Mat image_ori = cv::imread(file_image);
        cv::Mat image = image_ori.clone();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        GUtils::LoadPlyRGBPoints(cloud, file_cloud);

        cv::Mat depth;
        cv::flip(image_ori.t(), image_ori, -1); 
        if(virtualCamera.ProjectPointCloudToExistImage(cloud, image_ori, depth)){
            //cv::flip(image.t(), image, -1); 
            cv::flip(depth.t(), depth, -1); 
            cv::flip(image_ori.t(), image_ori, -1); 
            cv::imshow("Projected image", image_ori);
            cv::imshow("Original image", image);
            //cv::imshow("Projected depth", depth);
            while(cv::waitKey(10) != 'c'){
                
            }
        }      
    }

}

int main(int argc, char** argv)
{
    cout << " [GLSLAM] lidar slam using Neuvition solid state lidar device. \n";
    cout << "  build by Ye LIU, UTOPA, GuangZhou. \n\n";

    ConstumeProjector(argc, argv);

    //string fileName = FilePath + to_string(1) + ".pcd";
    //NormalEstimation::TestNormalEstimation(fileName);

    return 0;
}



// failed test: PCL range image 
/*
std::shared_ptr<VirtualCamera> virtualCamera;

void ReceiveLidarData()
{
    int i = 1;
    while (i < 1951) {
        string fileName = FilePath + to_string(i) + ".pcd";

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        GUtils::LoadPcdRGBAPoints(cloud_ptr, fileName);

        virtualCamera->SetPointCloud(cloud_ptr);

        cout << endl;
        i = i + 1;
    }
}

void RangeImageAndViewer()
{
    virtualCamera->ProjectionPclRangeImage();
}

void PclProjector()
{
    virtualCamera.reset(new VirtualCamera());

    std::thread thd_ReceiveLidar(ReceiveLidarData);
    std::thread thd_Viewer(RangeImageAndViewer);

    thd_ReceiveLidar.join();
    thd_Viewer.join();
}
*/

