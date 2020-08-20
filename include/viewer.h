#ifndef GL_VIEWER_H
#define GL_VIEWER_H

#include "SimpleIcpSLAM.h"
#include "GSegmentation.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pangolin/pangolin.h>

void DrawCamera(float mCameraSize)
{
    //float mCameraSize = 0.6;
    const float w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

}

void DrawKeyframe(Eigen::Matrix4d Twc, float mCameraSize)
{
    pangolin::OpenGlMatrix currentT;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            currentT.m[4*j + i] = Twc(i,j);
        }
    }

    glPushMatrix();
    glMultMatrixd(currentT.m);

    DrawCamera(mCameraSize);

    glPopMatrix();
}

pangolin::OpenGlMatrix TransformToOpenGL(Eigen::Matrix4d Twc)
{
    pangolin::OpenGlMatrix currentT;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            currentT.m[4*j + i] = Twc(i,j);
        }
    }

    return currentT;
}

void GLidarViewer(std::shared_ptr<GLidarSLAM> gLidarSLAM)
{

    // create pangolin window and plot the map
    pangolin::CreateWindowAndBind("Map Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0.7, -1.8, 0, 0, 0, 0.0, 1.0, 0.0)
    );

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<int> menu_sizepoint("menu. Point Size",2,1,10);
    pangolin::Var<bool> menu_showcamera("menu. Show Camera",true,true);
    pangolin::Var<float> menu_sizecamera("menu. Camera Size",0.5,0.1,2.0);
    pangolin::Var<bool> menu_followcamera("menu. Follow Camera",false,true);

    pangolin::Var<bool> menu_showChanged("menu. Show Move Objects",true,true);
    pangolin::Var<int> menu_sizechangedpoint("menu. Move Point Size",10,1,15);

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    bool bFollow = false;

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);        

        // draw poses
        glColor3f(0.0f,1.0f,0.0f);
        glLineWidth(2);
        std::vector<Eigen::Matrix4d> vPoses;
        if(gLidarSLAM->GetPoses(vPoses) && menu_showcamera){
            for(size_t i = 0; i < vPoses.size(); ++i){        
                DrawKeyframe(vPoses[i], menu_sizecamera);
            }
        }

        if(menu_followcamera && bFollow)
        {
            s_cam.Follow(TransformToOpenGL(vPoses[vPoses.size()-1]));
        }
        else if(menu_followcamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.1,-0.5, 0,0,0,0.0,1.0, 0.0));
            s_cam.Follow(TransformToOpenGL(vPoses[vPoses.size()-1]));
            bFollow = true;
        }
        else if(!menu_followcamera && bFollow)
        {
            bFollow = false;
        }

        // points
        glPointSize(menu_sizepoint);
        glBegin(GL_POINTS);
        gLidarSLAM->m_map.lock();
        int pointsize = gLidarSLAM->cloud_map.points.size();
        for(int i = 0; i < pointsize; ++i)
        {
            float r = gLidarSLAM->cloud_map.points[i].r/255.0;
            float g = gLidarSLAM->cloud_map.points[i].g/255.0;
            float b = gLidarSLAM->cloud_map.points[i].b/255.0;
            glColor3f(r, g, b);
            glVertex3d(gLidarSLAM->cloud_map.points[i].x,
                       gLidarSLAM->cloud_map.points[i].y,
                       gLidarSLAM->cloud_map.points[i].z);
        }
        gLidarSLAM->m_map.unlock();
        glEnd();

        if(menu_showChanged){
            glPointSize(menu_sizechangedpoint);
            glBegin(GL_POINTS);
            gLidarSLAM->m_changedPoint.lock();
            int pointsize = gLidarSLAM->cloud_changedPoint.points.size();
            for(int i = 0; i < pointsize; ++i){
                glColor3f(1.0, 0.0, 0.0);
                glVertex3d(gLidarSLAM->cloud_changedPoint.points[i].x,
                           gLidarSLAM->cloud_changedPoint.points[i].y,
                           gLidarSLAM->cloud_changedPoint.points[i].z);
            }
            gLidarSLAM->m_changedPoint.unlock();
            glEnd();
        }


        pangolin::FinishFrame();
        usleep(10);
    }


}

// cloud contain the points of the extracted plane
// cloud_color contain the points rest
void GLidarSegmentationViewer(std::shared_ptr<GSegmentation> gSegmentation)
//                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
//                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color
{
    // create pangolin window and plot the map
    pangolin::CreateWindowAndBind("Map Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0.7, -1.8, 0, 0, 0, 0.0, 1.0, 0.0)
    );

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<int> menu_sizepoint("menu. Point Size",2,1,10);
    pangolin::Var<bool> menu_showplane("menu. Show Plane",true,true);
    pangolin::Var<float> menu_sizecamera("menu. Camera Size",0.5,0.1,2.0);
    //pangolin::Var<bool> menu_followcamera("menu. Follow Camera",false,true);

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);        

     
        if(gSegmentation->segmented){
        gSegmentation->m_show_clouds.lock();
        // points inliers
        if(menu_showplane){
            glPointSize(menu_sizepoint);
            glBegin(GL_POINTS);
            for (std::size_t i = 0; i < gSegmentation->cloud_plane->points.size(); ++i)
            {
                int index = i;
                glColor3f(1.0, 0, 0);
                glVertex3d(gSegmentation->cloud_plane->points[index].x,
                           gSegmentation->cloud_plane->points[index].y,
                           gSegmentation->cloud_plane->points[index].z);
            }
            glEnd();
        }

        // points inliers
        glPointSize(menu_sizepoint);
        glBegin(GL_POINTS);
        for (std::size_t i = 0; i < gSegmentation->cloud_other->points.size(); ++i)
        {
            int index = i;
            float r = gSegmentation->cloud_other->points[index].r/255.0;
            float g = gSegmentation->cloud_other->points[index].g/255.0;
            float b = gSegmentation->cloud_other->points[index].b/255.0;
            glColor3f(r, g, b);
            glVertex3d(gSegmentation->cloud_other->points[index].x,
                       gSegmentation->cloud_other->points[index].y,
                       gSegmentation->cloud_other->points[index].z);
        }
        glEnd();
        gSegmentation->m_show_clouds.unlock();
        }

        pangolin::FinishFrame();
        usleep(10);
    }


}

/*
// PCL viewer failed with segmentation fail
int test_pcl_viewer(string FilePath)
{
    cout << " [GLSLAM] lidar slam using Neuvition solid state lidar device. \n";
    cout << "  build by Ye LIU, UTOPA, GuangZhou. \n\n";

    GLidarSLAM gLidarSLAM;

    pcl::visualization::PCLVisualizer viewer ("GLSLAM TEST");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(gLidarSLAM.cloud_map_ptr, 255, 255, 255);
    viewer.addPointCloud (gLidarSLAM.cloud_map_ptr, cloud_color_handler, "cloud_map");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_map");

    viewer.addCoordinateSystem (1.0, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0. 
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    //viewer.setPosition(800, 400); // Setting visualiser window position

    int i = 1;
    // Display the visualiser until 'q' key is pressed or go to the end of dataset
    while (!viewer.wasStopped ()) {

        string fileName = FilePath + to_string(i) + ".pcd";
        gLidarSLAM.AddFrameFile(fileName);
 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_ptr = gLidarSLAM.cloud_map.makeShared();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_ht(cloud_map_ptr, 255, 255, 255);
        viewer.updatePointCloud (cloud_map_ptr, cloud_color_ht, "cloud_map");

        viewer.spinOnce (100);
        i = i + 5;
    }

    return 0;
}
*/
#endif
