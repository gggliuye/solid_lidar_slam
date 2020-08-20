#include "PointCloudProjector.h"



VirtualCamera::VirtualCamera()
{
    bCameraSet = false;
}

void VirtualCamera::SetCameraIntrinsic(float fx_, float fy_, float cx_, float cy_, int rows_, int cols_)
{
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
    rows = rows_;
    cols = cols_;
    bCameraSet = true;
}

UVandZ VirtualCamera::ProjectPointToImage(float x, float y, float z)
{
    UVandZ result;
    float inv_z = 1/(z);
    result.u = (fx * x) * inv_z + cx;
    result.v = (fy * y) * inv_z + cy;
    result.z = z;
    return result;
}

bool VirtualCamera::PointInRange(UVandZ vuz)
{
    if(vuz.z < 0){
        return false;
    }
    if(vuz.u < 0 || vuz.u > rows-1){
        return false;
    }
    if(vuz.v < 0 || vuz.v > cols-1){
        return false;
    }
    return true;
}

bool VirtualCamera::ProjectPointCloudToImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat &image, cv::Mat &depth)
{
    if(!bCameraSet){
        return false;
    }

    image = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0,0,0));
    depth = cv::Mat(rows, cols, CV_32FC1, cv::Scalar(-1));

    int SpeadPixel = 3;

    int pointsize = cloud->points.size();
    std::cout << " points count : " << pointsize << std::endl;
    for(int i = 0; i < pointsize; ++i)
    {
        int r = cloud->points[i].r;
        int g = cloud->points[i].g;
        int b = cloud->points[i].b;
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
        //printf("x: %f, y : %f, z : %f. \n",x, y, z);
        UVandZ projected = ProjectPointToImage(x, y, z);
        //printf("u: %d, v : %d, z : %f. \n",projected.u, projected.v, projected.z);
        if(!PointInRange(projected)){
            continue;
        }

        cv::circle(image, cv::Point2f(projected.v, projected.u), SpeadPixel, cv::Scalar(b,g,r), -1);
        cv::circle(depth, cv::Point2f(projected.v, projected.u), SpeadPixel, cv::Scalar(z/100), -1);
/*
        depth.at<float>(projected.u, projected.v) = z;
        image.at<cv::Vec3b>(projected.u, projected.v)[0] = b;
        image.at<cv::Vec3b>(projected.u, projected.v)[1] = g;
        image.at<cv::Vec3b>(projected.u, projected.v)[2] = r;
*/
    }

    return true;
}

bool VirtualCamera::ProjectPointCloudToExistImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat &image, cv::Mat &depth)
{
    if(!bCameraSet){
        return false;
    }

    depth = cv::Mat(rows, cols, CV_32FC1, cv::Scalar(-1));

    int SpeadPixel = 3;

    int pointsize = cloud->points.size();
    std::cout << " points count : " << pointsize << std::endl;
    for(int i = 0; i < pointsize; i++)
    {
        int r = cloud->points[i].r;
        int g = cloud->points[i].g;
        int b = cloud->points[i].b;
        float x = cloud->points[i].x + offset_x;
        float y = cloud->points[i].y + offset_y;
        float z = cloud->points[i].z + offset_z;
        //printf("x: %f, y : %f, z : %f. \n",x, y, z);
        UVandZ projected = ProjectPointToImage(x, y, z);
        //printf("u: %d, v : %d, z : %f. \n",projected.u, projected.v, projected.z);
        if(!PointInRange(projected)){
            continue;
        }

        cv::circle(image, cv::Point2f(projected.v, projected.u), SpeadPixel, cv::Scalar(b,g,r), -1);
        cv::circle(depth, cv::Point2f(projected.v, projected.u), SpeadPixel, cv::Scalar(z/100), -1);
/*
        depth.at<float>(projected.u, projected.v) = z;
        image.at<cv::Vec3b>(projected.u, projected.v)[0] = b;
        image.at<cv::Vec3b>(projected.u, projected.v)[1] = g;
        image.at<cv::Vec3b>(projected.u, projected.v)[2] = r;
*/
    }

    return true;
}

// all tests with range image failed 
void VirtualCamera::ProjectionPclRangeImage()
{
    while(!bCloudUpdated){
        std::cout << " Waiting for the first frame. \n";
    }
    m_b_updated.lock();
    bCloudUpdated = false;
    m_b_updated.unlock();

    // set the virtual camera pose
    Eigen::Affine3f scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);  
/*
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud.sensor_origin_[0],
                                                             cloud.sensor_origin_[1],
                                                             cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (cloud.sensor_orientation_);
*/
    // set range image parameters
    float noise_level = 0.05;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage range_image;  

    // angle resolution 
    float angular_resolution = pcl::deg2rad(0.5f);
  
    // coordinate frame
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

    // create range image
    range_image.createFromPointCloud(cloud, 
                         angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                         scene_sensor_pose, coordinate_frame, 
                         noise_level, min_range, border_size);

    std::cout << range_image << "\n";

    // visualization PCL
    pcl::visualization::PCLVisualizer viewer("Range Image Viewer");
    viewer.setBackgroundColor(1, 1, 1);

    pcl::RangeImage::Ptr range_image_ptr = range_image.makeShared();
    // set range image handler and add range image to the viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

    viewer.initCameraParameters();
  
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image);
  
    while (!viewer.wasStopped ())
    {
        range_image_widget.spinOnce ();
        viewer.spinOnce ();
        pcl_sleep(0.01);
    
        m_b_updated.lock();
        bool live_update = bCloudUpdated;
        m_b_updated.unlock();
        if(live_update)
        {
            // live update - update the range image according to the selected view in the 3D viewer.
            // scene_sensor_pose = viewer.getViewerPose();
            m_range_image.lock();
            range_image.createFromPointCloud (cloud, angular_resolution,
                                        pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                        scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
            range_image_widget.showRangeImage(range_image);
            m_range_image.unlock();

            m_b_updated.lock();
            bCloudUpdated = false;
            m_b_updated.unlock();
        }
    }

}

void VirtualCamera::SetPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in)
{
    m_range_image.lock();
    cloud += *cloud_in;
    m_range_image.unlock();

    m_b_updated.lock();
    bCloudUpdated = true;
    m_b_updated.unlock();
}


namespace NormalEstimation
{

//  only used for organized point cloud 
//  it is faster as organized point cloud offers the cloud point relationship
void TestNormalEstimationOrganized(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filename, *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // estimation methods : 
    //    COVARIANCE_MATRIX
    //    AVERAGE_3D_GRADIENT
    //    AVERAGE_DEPTH_CHANGE
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

void TestNormalEstimation(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filename, *cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);


    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);

    normalEstimation.setKSearch(20);
    //normalEstimation.setRadiusSearch(0.3);

    normalEstimation.compute(*normals);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 1, "normals");

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}


}  // namespace NormalEstimation

void FAILED::TestRangeImageSimple()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;

    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
        for (float y=-0.5f; y<=0.5f; y+=0.01f)
        {
            pcl::PointXYZ point;  
	    point.x = x;  point.y = y;  point.z = 2.0f - y;
            point_cloud.points.push_back (point);
        }
    }
    point_cloud.width = (int) point_cloud.points.size ();  
    point_cloud.height = 1;
/*
    // set the virtual camera pose
    Eigen::Affine3f scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);  

    // set range image parameters
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage range_image;  

    // angle resolution 
    float angular_resolution = pcl::deg2rad(0.5f);
  
    // coordinate frame
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

    // create range image
    range_image.createFromPointCloud(point_cloud, 
                         angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                         scene_sensor_pose, coordinate_frame, 
                         noise_level, min_range, border_size);

    //std::cout << range_image << "\n";
*/
}


void FAILED::TestOffical(std::string filename)
{
typedef pcl::PointXYZ PointType;

float angular_resolution = pcl::deg2rad(0.5f); 
float support_size = 0.2f; 
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; 
bool setUnseenToMaxRange = false;

  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>); 
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr; 
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges; 
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ()); 

  if(true)
  {
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1) 
    {
      cerr << "Was not able to open file \""<<filename<<"\".\n";
      return;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
  }
  else 
  {
    setUnseenToMaxRange = true;
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  
	point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  
		point_cloud.height = 1;
  }
  

  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage); 
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size); 
  //range_image.integrateFarRanges (far_ranges); 
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();

  std::cout << range_image << "\n";

  pcl::visualization::PCLVisualizer viewer ("3D Viewer"); 
  viewer.setBackgroundColor (1, 1, 1); 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image"); 
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  //setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  

  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (range_image);
  
  pcl::RangeImageBorderExtractor range_image_border_extractor; 
  pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
  //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;
  
  pcl::PointCloud<int> keypoint_indices; 
  narf_keypoint_detector.compute (keypoint_indices); 
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

  //show keypoints in range_image_widget
  //for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
                                  //keypoint_indices.points[i]/range_image.width);
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr; 
  keypoints.points.resize (keypoint_indices.points.size ()); 
  for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  // -----Main loop-----
  while (!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }

}
