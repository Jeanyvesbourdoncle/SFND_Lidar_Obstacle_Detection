// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction and region based filtering
    // Step 1 : Create the filtering object
    pcl:: VoxelGrid<PointT> vox;
    // definition of the new point cloud
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vox.setInputCloud (cloud); // input Cloud
    vox.setLeafSize (filterRes, filterRes,filterRes); // cube definition : lenght, width, height
    vox.filter (*cloudFiltered); // new point cloud with the stored results in this cloud 
    // After this state, we are sur that we are on voxel pixel (volume pixel) in one cube

    // Step 2 : Crop Box creation, input : the cloudFiltered = 1 voxel pixel in one cube
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint); // minimum voxel in the region
    region.setMax(maxPoint); // maximum voxel in the region
    region.setInputCloud(cloudFiltered); // input Cloud (with the voxel) 
    region.filter (*cloudRegion); // output cloud with the dedicated region


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud <PointT>());

  // plane cloud generation
    for (int index : inliers -> indices)
        planeCloud->points.push_back(cloud->points[index]);

    // obstacle cloud generation : Extraction from the inliers to create the obstacles
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while (maxIterations--)
	{
		//randomly pick two points
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert (rand()%(cloud->points.size()));
		float x1,y1,z1,x2,y2,z2,x3,y3,z3;

		auto itr = inliers.begin();
		x1 = cloud -> points[*itr].x;
		y1 = cloud -> points[*itr].y;
		z1 = cloud -> points[*itr].z;
		itr++;
		x2 = cloud -> points[*itr].x;
		y2 = cloud -> points[*itr].y;
		z2 = cloud -> points [*itr].z;
		itr++;
		x3 = cloud -> points [*itr].x;
		y3 = cloud -> points [*itr].y;
		z3 = cloud -> points [*itr].z;
		
		float a = ((y2 - y1)*(z3-z1)) - ((z2-z1)*(y3-y1));
		float b = ((z2 - z1)*(x3-y1)) - ((x2-x1)*(z3-z1));
		float c = ((x2 - x1)*(y3-y1)) - ((y2-y1)*(x3-x1));
		float d = -((a*x1) +(b*y1)+(c*z1));

		

		for ( int index =0; index <cloud ->points.size(); index++)
		{
			if (inliers.count (index)>0)
			continue;

			//pcl::PointXYZ point = cloud->points[index];
			PointT point = cloud -> points[index];
            float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float distance  =fabs (a*x4+b*y4+c*z4+d) /sqrt (a*a+b*b+c*c);
			if (distance <= distanceTol)
				inliers.insert(index);

		}

		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> (cloudOutliers, cloudInliers);
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
	pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients :: Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers -> indices.size()==0)
    { 
        std::cout << "could not estimate a planar model for the given dataset"<< std::endl;
    }



    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //PCL Search creation en utilisant le Template pointT
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    
    // feed the algorithme in the cloud (argument dans la fonction clustering)
    tree->setInputCloud(cloud);

    //indices cluster creation as written in the documentation 
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    // use of the cluster tolerance,minSize,maxSize defines in the argument header for the ec object
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    // tell the hyperparameters in the tree
    ec.setSearchMethod(tree);
    // set the cloud
    ec.setInputCloud(cloud);
    // generate the cluster indices by calling the extract method
    ec.extract(clusterIndices);

    //set all to go through the cluster indices and create some point clouds in each point cloud's going to be a different cluster 
    // for loop : iterate through cluster indices and the type for each of those is as pcl point indices
    //pcl::PointIndices : vector of ocl indices

    for(pcl::PointIndices getIndices : clusterIndices)
    { 
        // creation of a new cloud cluster 
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        //iterate through the indices from getIndices.indices and tack each of those + push it into the obstacle cloud  cluster just created
        for (int index : getIndices.indices)
            cloudCluster->points.push_back (cloud ->points[index]);
        
        // set up the width and height for this obstacle cloud
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        //push back into my clusters : vector of point clouds
        clusters.push_back(cloudCluster);

    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper (int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector <bool>& processed, KdTree* tree, float distanceTol)
{
	// mark the point as being processed
	processed[indice] =true;
	// push the point back into clusters
	cluster.push_back(indice);

	// which points are near by this indice
	// the tree help us with this, we call tree search and we do for points indice + distance tolerance
	// we grab that point an say "what's nearby within this distance tolerance" and give us a list of indices that are nearby
	std::vector<int> nearest = tree->search (cloud -> points[indice], distanceTol);

	// iteration through those nerby indices
	for (int id : nearest)
	{
		//if that point, that id hasn't been processed yet, then we just include it into cluster helper.
		if(!processed[id])
			{clusterHelper(id,cloud,cluster,processed,tree,distanceTol);}
	}
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster
(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree,float distanceTol, int minSize, int maxSize) 
{

	// return list of indices for each cluster
	// vector of ints : cluster
	std::vector<typename pcl::PointCloud<PointT> ::Ptr> clusters;

	// keep track of which points have been processed or not
	// creation a vector of booleans, it's going to be exactly the same size as the point size : all te data for the 2D set : 11 points
	std :: vector <bool> processed (cloud->points.size(),false);
    
    for (int idx =0; idx < cloud->points.size(); idx++)
        {
            if (processed[idx]==false)
                {
                    std::vector<int> clusterIdx;
                    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
                    clusterHelper (idx,cloud,clusterIdx, processed,tree,distanceTol);
                    if (clusterIdx.size() >= minSize && clusterIdx.size() <= maxSize)
                    {
                        for (int i=0; i<clusterIdx.size(); i++)
                        {
                            cluster->points.push_back(cloud->points [clusterIdx[i]]);
                        }
                        cluster->width = cluster->points.size();
                        cluster->height =1;
                        clusters.push_back(cluster);
                    }
                    else
                    {
                        for (int i=1; i<clusterIdx.size();i++)
                        {
                            processed[clusterIdx[i]] = false;
                        }
                    }
                    
                }
        } 

	return clusters;

} 



template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}