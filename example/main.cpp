#include <iostream>
#include <thread>
#include <fstream>

#include "viewer.h"

using namespace std;

std::shared_ptr<GLidarSLAM> gLidarSLAM;

string FilePath = "/home/viki/UTOPA/Neuvition/Data/point_clouds/";

static bool compare_timestamp(const int64_t &r1, const int64_t &r2) {
    return r1 < r2;
}

void ReceiveLidarData()
{
/*
    int i = 350;
    while (i < 1951) {
        cout << "FRAME " << i << endl;
        string fileName = FilePath + to_string(i) + ".pcd";
        gLidarSLAM->AddFrameFile(fileName);
        cout << endl;
        i = i + 1;
    }
*/

    std::vector<int64_t> vTimestamps;
    {
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (FilePath.c_str())) == NULL) {
            std::cerr << "[ERROR] cannot open images folder ! " << std::endl;
            return;
        }
        while ((ent = readdir (dir)) != NULL) {
            std::string pathimg = FilePath + ent->d_name;
            std::string name_t = ent->d_name;
            std::string file_name = name_t.substr(0, 16);

            std::stringstream a;
            a << file_name;
            int64_t time_stamp = 0;
            a >> time_stamp;

            //int time_stamp = atoi(file_name.c_str());
            if(pathimg.length() - FilePath.length() < 4){
                continue;
            }

            vTimestamps.push_back(time_stamp);
            //std::cout << file_name << " " << time_stamp << " " << pathimg << std::endl;
        }
    }

    std::sort(vTimestamps.begin(), vTimestamps.end(), compare_timestamp);

    for(size_t i = 0 ; i < vTimestamps.size() ; i ++){
        std::cout << "\nProcess cloud with time stamp : " << vTimestamps[i] << "\n" ;
        std::string file_cloud = FilePath + std::to_string(vTimestamps[i]) + ".ply";
        gLidarSLAM->AddFrameFile(file_cloud);
    }

}

void Viewer()
{
    GLidarViewer(gLidarSLAM);
}

int main()
{
    cout << " [GLSLAM] lidar slam using Neuvition solid state lidar device. \n";
    cout << "  build by Ye LIU, UTOPA, GuangZhou. \n\n";

    gLidarSLAM.reset(new GLidarSLAM());

    std::thread thd_ReceiveLidar(ReceiveLidarData);
    std::thread thd_Viewer(Viewer);

    thd_ReceiveLidar.join();
    thd_Viewer.join();

    return 0;
}

