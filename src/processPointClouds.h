// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/kdtree/kdtree.h>
#include "render/box.h"

/*------------------------------------------------------------------------------------------------------------------------------------
Import from quiz/kdTree.h : 
Insert and the insert helper : for the insertion of a new point into the tree
Search and search helper : for the search of the nearby points inside the three compared to a given target point  
-------------------------------------------------------------------------------------------------------------------------------------*/



// Structure to represent node of kd tree
struct Node
{
	//std::vector<float> point;
	pcl::PointXYZI point;
  int id; //unique Id points 
	Node* left; // left to the node
	Node* right; // right to the node

	// function for creating a brand new node, when we can give in this brand new point in a band new Id
    Node (pcl ::PointXYZI arr, int setId) 
    : point (arr), id (setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

// take a brand new point
	// point representation : vector of floats : the first element of the vetor is going to be your x component and the second element going to be the y component.
	// Id = unique identifier for the point : just an index that it references in the point cloud

	// insert fonction is a recursive 
  // the target is to run the tree to reach the Node Null, when the node take an Id, the NULL node is transfered in the depht



void insertHelper( Node** node, uint depth, pcl::PointXYZI point, int id)
  {
    // verification if the the tree is empty : verification if le node is NULL, *node est ce que la racine doit commencer
    if (*node == nullptr) { // // we deference it to actually see what it's value is, in this case : Root pointer : new data that you should be poiting at now
      *node = new Node(point, id); // // reasign this node in the tree  + assign this brand new node
    }
    else {
      // Traversing the tree
      // Calculation of the current dim (3D kd-tree)

      uint cd = depth % 3; // 3D kd tree
      // cd = 0 ou 1, if cd=0 : look at the x-values, if cd=1 : look at the y-values
      if (cd == 0) {
        if (point.x < (*node)->point.x) {
          insertHelper(&(*node)->left, depth + 1, point, id);
        }
        else {
          insertHelper(&(*node)->right, depth + 1, point, id);
        }
      }
      else {
        if (point.y < (*node)->point.y) {
          insertHelper(&(*node)->left, depth + 1, point, id);
        }
        else {
          insertHelper(&(*node)->right, depth + 1, point, id);
        }
      }
    } // insertHelper is terminate when its hits a null node
  }


  /// Insertion of a new point in the tree
  /// Point XYZI :  2D point represented by a vector containing two floats
  /// int id : Point identifier
  void insert(
    pcl::PointXYZI point,
    int            id)
  {
    // Insertion of a new point in the tree.
    // Creation of a new node + correct integration in accordance in the root
    // call insertHelper : recursive fonction : able to reassign this node in the tree.
		// &root : passing the memory adresse for root, to start to "root" 
		// depth start to 0,
    insertHelper(&root, 0, point, id);
  }


  void searchHelper( pcl::PointXYZI  target, Node* node, const int depth, const float&  distanceTol, std::vector<int>& ids)
  {
    if (node != nullptr) {
      if ((node->point.x >= target.x - distanceTol &&  // Check for x-axis
           node->point.x <= target.x + distanceTol) &&
          (node->point.y >= target.y - distanceTol &&  // Check for y-axis
           node->point.y <= target.y + distanceTol)) {
        float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) +
                              (node->point.y - target.y) * (node->point.y - target.y));

        if (distance <= distanceTol) {
          ids.push_back(node->id);
        }
      }

      // 3D kd-tree
      if (depth % 3 == 0) {

        if ((target.x - distanceTol) < node->point.x) {
          searchHelper(target, node->left, depth + 1, distanceTol, ids);
        }

        if ((target.x + distanceTol) > node->point.x) {
          searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
      }
      else {

        if ((target.y - distanceTol) < node->point.y) {
          searchHelper(target, node->left, depth + 1, distanceTol, ids);
        }

        if ((target.y + distanceTol) > node->point.y) {
          searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
      }

    }
  }


  // Return a list of point ids in the tree that are within distance of target
  std::vector<int> search( pcl::PointXYZI target, const float&   distanceTol)
  {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);

    return ids;
  }
};





/*--------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------*/




template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    // Add Ransac --> origin: quiz/ Ransac.cpp
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    // Add EucledianCLuster --> origin : quiz/Cluster.cpp
	  std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);
	
    // Add clusterHelper --> origin : quiz/Cluster.cpp
    void clusterHelper (int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector <bool>& processed, KdTree* tree, float distanceTol);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */