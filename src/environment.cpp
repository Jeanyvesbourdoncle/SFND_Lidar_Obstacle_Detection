/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    //bool renderScene = true;
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor object (heap)
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer,lidar->position, inputCloud);
    //renderPointCloud (viewer, inputCloud,"inputCloud");

    //renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
  
    // For the segmentation : 2 opportunities are possible : call the SegmentPlane or the RansacPlane

    // Create point processor object
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.RansacPlane(inputCloud, 100, 0.2);
    
    // Segmentation : Obstacle Cloud / Road Cloud
    renderPointCloud (viewer,segmentCloud.first, "obstCloud", Color (1,0,0));
    renderPointCloud (viewer,segmentCloud.second, "planeCloud", Color (0,1,0));
    
    


    // Call the cluster fonction + Rendering the results with the PCL viewer
    // Call the processor clustering function (input : segmentCloud.first = obstacle Cloud and segmentCloud.second =road Cloud)
    // hyperparameters depends on the data  : 1.0 for the distance tolerance (the points must be within one distance unit)
    // 3 = three points to be considered a cluster, 30 = max point to define cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ> :: Ptr> cloudClusters = pointProcessor.Clustering (segmentCloud.first, 1.0, 3, 30);
    int clusterId=0;
    std::vector<Color> colors = {Color (1,0,0), Color(1,1,0), Color(0,0,1)};
    // iterate through the vector of points clouds
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
            // Size of the cluster + visualization of the obstacle with differents colors
            std::cout <<"cluster size";
            pointProcessor.numPoints (cluster);
            renderPointCloud(viewer,cluster, "obstCloud"+ std::to_string(clusterId), colors[clusterId % colors.size()]);
        
            // Box creation around the 3 clusters 
            Box box = pointProcessor.BoundingBox (cluster);
            renderBox(viewer,box,clusterId);
        
            clusterId++;
    }
}



void cityBlock (pcl::visualization::PCLVisualizer::Ptr& viewer)
{
// Pipeline - Step 1 : Open 3D viewer and display a single PCD file
ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd ("../src/sensors/data/pcd/data_1/0000000015.pcd");
//renderPointCloud(viewer,inputCloud, "Input cloud"); // Visualization of the Input Cloud, it's our start of the pipeline

// Pipeline - Step 2 : Render the results after the filtering state
pcl::PointCloud<pcl::PointXYZI>::Ptr regionCloud = pointProcessorI ->FilterCloud(inputCloud,0.3, Eigen::Vector4f (-15,-6,-3,1), Eigen::Vector4f (35,7,2,1));
renderPointCloud(viewer,regionCloud, "regional cloud"); // Visualization of the region of interests after a voxel grid filtering

// Pipeline - Step3 : Render the results after the segmentation. The resulats are 2 clouds : road and obstacle
std::pair <pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(regionCloud,110,0.2);
//renderPointCloud (viewer,segmentCloud.first, "obstCloud", Color (1,0,0)); // Visualization of the obstacle alone in red
//renderPointCloud (viewer,segmentCloud.second, "planeCloud", Color (0,1,0)); // Visualization of the road (without obstacle) in green

// Pipeline - Step 4 : Render the results after the euclidean clustering 
// Create a  KdTree object
KdTree* tree = new KdTree;
// Insert the point from the obstacle Point Cloud in the tree   
for (int i=0; i<segmentCloud.first->points.size(); i++)
    {tree ->insert(segmentCloud.first->points[i],i);}

// Selection of the Obstacle Point Cloud to apply the euclideanCloud 
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, tree, 0.3, 20, 500);

int clusterId=0;
std::vector<Color> colors = {Color (1,0,0), Color(1,1,0), Color(0,0,1)}; // Red, Red/Green, Blue
// Iterate through the vector of points clouds
for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
            // Size of the cluster + visualization of the obstacle with differents colors
            std::cout <<"cluster size";
            pointProcessorI -> numPoints (cluster);
            //renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterId),colors[clusterId]);
            
            // Bounding Box creation around the cluster
            Box box = pointProcessorI->BoundingBox (cluster);
            renderBox(viewer,box,clusterId);
            
            clusterId++;
    } 

    delete tree; 
}






//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(
  CameraAngle                             setAngle,
  pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor (0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch(setAngle)
  {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle!=FPS) {
    viewer->addCoordinateSystem(1.0);
  }
}



int main (int argc, char** argv)
{
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  //simpleHighway(viewer); // Call the highway with  the 3 clusters (11 points)
  cityBlock(viewer); // Call the real simulation
  while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
    
}


