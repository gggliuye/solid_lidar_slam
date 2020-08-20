#include "SimpleIcpSLAM.h"

GLidarSLAM::GLidarSLAM()
{
    cloud_map_ptr = cloud_map.makeShared();
    cloud_changed_ptr = cloud_changed.makeShared();
    transformation_matrix_initial = Eigen::Matrix4d::Identity();
}

void GLidarSLAM::ReSet()
{
    frameCount = 0;
    firstFlag = true;
    vPcdFilePaths.clear();
    vRelativeTransformations.clear();
    vSensorTransformations.clear();
}


// The problem with ICP is that it is sensitive to the initial gauss
// if we have not good initial gauss, we have to make sure the device only have small move and small rotation
// As a result, it demand another sensor source to find a good initial gauss 
//     -- for example : visual odometry, IMU sensor, or pose using Feature matching methods
//  * setTransformationEstimation: set estimation method
//  * addCorrespondenceRejector(const CorrespondenceRejectorPtr &rejector): outlier rejection
//  * setUseReciprocalCorrespondences(bool use_reciprocal_correspondence): bi-direction check
bool GLidarSLAM::MatchSimpleICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_old, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int iterations)
{
    // a rotation matrix and translation vector
    // Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);

    *cloud_tmp = *cloud_new;

    pcl::console::TicToc time;
    time.tic();
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    //pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // it is a divergence threshold
    //icp.setEuclideanFitnessEpsilon(1);
    icp.setTransformationEpsilon(1e-9);
    //icp.setUseReciprocalCorrespondences(true);
    // should be approx. 1/100 of ransacThreshold
    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setMaximumIterations (iterations);
    icp.setRANSACOutlierRejectionThreshold(ransacThreshold);

    icp.setInputSource (cloud_tmp);
    icp.setInputTarget (cloud_old);
    icp.align (*cloud_tmp, transformation_matrix.cast<float>());

    if(!icp.hasConverged ()){
        PCL_ERROR ("\nICP has not converged.\n");
        return false;
    } else {
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        if(VERBOSE){
            std::cout << "ICP has converged, score is " << icp.getFitnessScore () << ", in " 
                      << time.toc () << " ms" << std::endl;
            //cout << transformation_matrix << endl << endl;
        }
        return true;
    }

}

bool GLidarSLAM::MatchPlaneICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int iterations)
{
    // a rotation matrix and translation vector
    // Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);

    *cloud_tmp = *cloud_new;

    pcl::console::TicToc time;
    time.tic();

    // calcualte normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_tmp);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_tmp(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(kdtree_tmp);
    normalEstimation.setKSearch(10);
    normalEstimation.compute(*normals);

    // combine the normals and the points
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal(new pcl::PointCloud<pcl::PointNormal>);
    CombinePointAndNormal(cloud_tmp, normals, pointNormal);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal_old_ptr(new pcl::PointCloud<pcl::PointNormal>);
    cloud_normal_old_ptr = cloud_normal_old.makeShared();

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    // need normal offered to use point to plane ICP
    pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr trans_lls(
                    new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
    icp.setTransformationEstimation (trans_lls);


/*
    // set tree search method
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud(cloud_tmp);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(cloud_old);
    icp.setSearchMethodSource(tree1);
    icp.setSearchMethodTarget(tree2);
*/

    // it is a divergence threshold
    //icp.setEuclideanFitnessEpsilon(1);
    icp.setTransformationEpsilon(1e-9);
    //icp.setUseReciprocalCorrespondences(true);
    // should be approx. 1/100 of ransacThreshold
    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setMaximumIterations (iterations);
    icp.setRANSACOutlierRejectionThreshold(ransacThreshold);

    icp.setInputSource (pointNormal);
    icp.setInputTarget (cloud_normal_old_ptr);

    icp.align (*pointNormal, transformation_matrix.cast<float>());


    cloud_normal_old = *pointNormal;

    if(!icp.hasConverged ()){
        PCL_ERROR ("\nICP has not converged.\n");
        return false;
    } else {
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        if(VERBOSE){
            std::cout << "ICP has converged, score is " << icp.getFitnessScore () << ", in " 
                      << time.toc () << " ms" << std::endl;
            //cout << transformation_matrix << endl << endl;
        }
        return true;
    }

}

bool GLidarSLAM::MatchNdtOmp(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_old, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int resolution, int iterations)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_tmp = *cloud_new;

    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr fast_ndt(
                   new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

    fast_ndt->setResolution(resolution);
    fast_ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);

    int max_threads = omp_get_max_threads();
    fast_ndt->setNumThreads(max_threads);
    // the minimal change of pose (use as LM stop criterion)
    //fast_ndt->setTransformationEpsilon(1e-4);
    // set the maximum step size of LM optimization
    //fast_ndt->setStepSize(0.1);
    // maximum iteration of LM
    fast_ndt->setMaximumIterations(iterations);

    fast_ndt->setInputSource(cloud_tmp);
    fast_ndt->setInputTarget(cloud_old);
    fast_ndt->align(*cloud_tmp, transformation_matrix.cast<float>());

    if(!fast_ndt->hasConverged ()){
        PCL_ERROR ("\nNDT has not converged.\n");
        return false;
    } else {
        transformation_matrix = fast_ndt->getFinalTransformation().cast<double>();
        if(VERBOSE){
            std::cout << "NDT has converged, score is " << fast_ndt->getFitnessScore () 
                      << ", in " << time.toc () << " ms" << std::endl;
        }
        //cout << transformation_matrix << endl;
        return true;
    }
}

bool GLidarSLAM::MatchNdt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_old, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new,
                    Eigen::Matrix4d &transformation_matrix, int resolution, int iterations)
{
    pcl::console::TicToc time;
    time.tic ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_tmp = *cloud_new;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
                   new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

    ndt->setResolution(resolution);
    ndt->setMaximumIterations(iterations);

    ndt->setInputSource (cloud_tmp);
    ndt->setInputTarget (cloud_new);
    ndt->align(*cloud_tmp, transformation_matrix.cast<float>());

    if(!ndt->hasConverged ()){
        PCL_ERROR ("\nNDT has not converged.\n");
        return false;
    } else {
        transformation_matrix = ndt->getFinalTransformation().cast<double>();
        if(VERBOSE){
            std::cout << "NDT has converged, score is " << ndt->getFitnessScore () 
                      << ", in " << time.toc () << " ms" << std::endl;
        }
        cout << transformation_matrix << endl;
        return true;
    }
}

bool GLidarSLAM::GetPoses(std::vector<Eigen::Matrix4d> &output)
{
    if(frameCount < 2)
        return false;

    m_pose.lock();
    output = vSensorTransformations;
    m_pose.unlock();
    return true;
}

void GLidarSLAM::AddFrameFile(string fileName)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZRGB>);
    LoadPlyRGBPoints(cloud_new, fileName);

    RandomDownSampleAndFilter(cloud_new, downSampleRate);

    AddFrame(cloud_new);
}

void GLidarSLAM::SetFirstFrameWithNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_first)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);

    *cloud_tmp = *cloud_first;

    // calcualte normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_tmp);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_tmp(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(kdtree_tmp);
    normalEstimation.setKSearch(10);
    normalEstimation.compute(*normals);

    // combine the normals and the points
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal(new pcl::PointCloud<pcl::PointNormal>);
    CombinePointAndNormal(cloud_tmp, normals, pointNormal);

    cloud_normal_old = *pointNormal;
}


void GLidarSLAM::AddFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new)
{
    if(firstFlag){
        cloud_old = *cloud_new;
        if(use_normal){
            SetFirstFrameWithNormal(cloud_new);
        }

        m_map.lock();
        cloud_map = *cloud_new;
        cloud_map_ptr = cloud_map.makeShared();
        m_map.unlock();

        firstFlag = false;
        frameCount = 1;
        return;
    }

    //transformation_matrix_initial = Eigen::Matrix4d::Identity();
    
    if(use_normal){
        MatchPlaneICP(cloud_new, transformation_matrix_initial, icpMaxIterations);
    } else {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_old_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_old_ptr = cloud_old.makeShared();
        MatchSimpleICP(cloud_old_ptr, cloud_new, transformation_matrix_initial, icpMaxIterations);
    }

    // update last frame
    cloud_old = *cloud_new;

    // save relative poses
    vRelativeTransformations.push_back(transformation_matrix_initial);
 
    // save the transformation from current to world(first frame)
    if(frameCount == 1){
        m_pose.lock();
        vSensorTransformations.push_back(transformation_matrix_initial);
        m_pose.unlock();
    } else {
        m_pose.lock();
        Eigen::Matrix4d tmp = vSensorTransformations[frameCount-2];
        Eigen::Matrix4d transformation_to_world = tmp * transformation_matrix_initial;
        vSensorTransformations.push_back(transformation_to_world);
        m_pose.unlock();
    }

    // transform the current point cloud to the world reference
    Eigen::Matrix4d tmp = vSensorTransformations[frameCount-1];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_new, *transformed_cloud, tmp);

    if(bTestChangedPoints){
        //CompareAndFindOutlier(cloud_old_ptr, cloud_new, transformation_matrix_initial, tmp);
    }

    // add the new point cloud to the point cloud map
    m_map.lock();
    if(frameCount%5 == 0){
        cloud_map = cloud_map + *transformed_cloud;
        cloud_map_ptr = cloud_map.makeShared();
        //RandomDownSampleRGB(cloud_map_ptr, 0.9);
    }
    m_map.unlock();

    frameCount++;
}

void GLidarSLAM::ConstumeRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seed,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grown)
{
    float distanceThreshold = 1.0f;
    
    // expand the changed points using KD Tree search
    // creates kdtree object
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    // sets our randomly created cloud as the input
    kdtree.setInputCloud(cloud_new);

    // K nearest neighbor search
    int neighbour_number = 20;
    int point_number = static_cast<int> (cloud_seed->points.size());

    std::vector<int> neighbours;
    std::vector<float> distances;

    std::vector<std::vector<int>> point_neighbours;
    std::vector<std::vector<float>> point_distances;
    point_neighbours.resize(point_number, neighbours);
    point_distances.resize(point_number, distances);

    // for each points search its kdtree neighbours
    for (int i_point = 0; i_point < point_number; i_point++)
    {
        neighbours.clear();
        distances.clear();
        if (!pcl::isFinite (cloud_seed->points[i_point]))
            continue;
        kdtree.nearestKSearch(cloud_seed->points[i_point], neighbour_number, neighbours, distances);
        point_neighbours[i_point].swap(neighbours);
        point_distances[i_point].swap(distances);
    }

    std::cout << " Find " << neighbour_number << " Kd tree neighbours. \n";
    
    std::vector<int> indicesAdded;
    cloud_grown->clear();
    // check and add the points
    for (int i = 0; i < point_number; i++){
        for(int k = 0 ; k < neighbour_number; k++){
            int index = point_neighbours[i][k];
            if(point_distances[i][k] > distanceThreshold)
                continue;
            if(CheckExistance(indicesAdded, index))
                continue;
            indicesAdded.push_back(index);
            pcl::PointXYZRGB pt = cloud_new->points[index];
            cloud_grown->points.push_back(pt);
        }
    }
    cloud_grown->width = cloud_grown->points.size();
    cloud_grown->height = 1;
    cloud_grown->is_dense = false;

    std::cout << " Have grown " << indicesAdded.size() << " points. \n";
    
}



int GLidarSLAM::CompareAndFindOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_old, 
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new,
                                      Eigen::Matrix4d transformation_matrix,
                                      Eigen::Matrix4d transformation_matrix_to_world)
{
    float octreeResolution = 0.6f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(octreeResolution);
 
    // add old point cloud
    octree.setInputCloud(cloud_old);  
    octree.addPointsFromInputCloud();   

    // switch buffer 
    octree.switchBuffers();

    // project the new pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_new, *transformed_cloud, transformation_matrix);

    // add the new point cloud
    octree.setInputCloud(transformed_cloud);
    octree.addPointsFromInputCloud();

    // retrieve the new points -> indices are index of the new point cloud
    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    int nFound = newPointIdxVector.size(); 

    // save the changed points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0; i < nFound; i++)
    {
       int index = newPointIdxVector[i];
       if(cloud_new->points[index].y < -fCarHeight)
           continue;
       pcl::PointXYZRGB pt = cloud_new->points[index];
/*
       pt.x = cloud_new->points[index].x;
       pt.y = cloud_new->points[index].y;
       pt.z = cloud_new->points[index].z;
*/
       cloud_tmp->points.push_back(pt);
    }
    cloud_tmp->width = cloud_tmp->points.size();
    cloud_tmp->height = 1;
    cloud_tmp->is_dense = false;

    //Use RadiusOutlierRemoval to remove the point whitch is far away to others
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(cloud_tmp);
    outrem.setRadiusSearch(1.0);
    outrem.setMinNeighborsInRadius(30);
    outrem.filter(*cloud_changed_ptr);     
  
    std::cout << " [Changed Detection] after filter " << cloud_changed_ptr->points.size() << " points new. \n";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grown_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    ConstumeRegionGrowing(cloud_new, cloud_changed_ptr, cloud_grown_ptr);

    // project the point to world frame to show in viewer
    m_changedPoint.lock();
    pcl::transformPointCloud(*cloud_grown_ptr, cloud_changedPoint, transformation_matrix_to_world);
    m_changedPoint.unlock();

    std::cout << " [Changed Detection] after growth: " << cloud_grown_ptr->points.size() << " points new. \n";

    return nFound;
}

