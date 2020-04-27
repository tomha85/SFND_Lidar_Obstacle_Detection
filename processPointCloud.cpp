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

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,Eigen::Vector4f minPoint,Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_nearremoved (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    
    auto startTime{std::chrono::steady_clock::now()};
    
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloud_downsampled);

    
    typename pcl::CropBox<PointT> region(true);
    region.setInputCloud(cloud_downsampled);
    region.setMin(Eigen::Vector4f(-2.0,-1.5,-2,1));
    region.setMax(Eigen::Vector4f(2.7,1.5,0,1));
    region.setNegative(true);
    region.filter(*cloud_nearremoved);

    
    typename pcl::CropBox<PointT> ro(true);
    ro.setInputCloud(cloud_nearremoved);
    ro.setMin(minPoint);
    ro.setMax(maxPoint);    
    ro.filter(*cloud_filtered);

    auto endTime(std::chrono::steady_clock::now());
    auto elapsedTime{std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime)};
    std::cout<<"filtering took" <<elapsedTime.count()<<"milliseconds"<<std::endl;
  
  return cloud_filtered; 
}

template<typename PointT> 
std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstCloud( new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud( new pcl::PointCloud<PointT>());

    for (int index:inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    pcl::ExtractIndices<PointT> ec;
    ec.setInputCloud(cloud);
    ec.setIndices(inliers);
    ec.setNegative(true);
    ec.filter(*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime(std::chrono::steady_clock::now());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizedCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setmethodType(pcl::SAC_RANSAC);
    seg.setMaxInterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);

    if(inliers->indices.size()==0)
        { 
           std::cout<<"Could not find model"<<std::endl;
        }

    auto endTime{std::chrono::steady_clock::now()};
    auto elapsedTime{std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime)};
    std::cout<<"plane segment took" << elapsedTime.count()<<"milliseconds"<<std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult=SeparateClouds(inliers,cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,int maxIterations, float distanceThreshold)
{
    auto startTime=std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    while(maxIterations--)
    {
    std::unordered_set<int> inliers;
    while (inliers.size()<3)
        inliers.insert(rand()%(cloud->points.size()));

    float x1,y1,z1,x2,y2,z2,x3,y3,z3;
    auto itr= inliers.begin();   

    x1=cloud->points[*itr].x;   
	y1=cloud->points[*itr].y;
	z1=cloud->points[*itr].z;

	++itr;

	x2=cloud->points[*itr].x;
	y2=cloud->points[*itr].y;
	z2=cloud->points[*itr].z;

	++itr;

	x3=cloud->points[*itr].x;
	y3=cloud->points[*itr].y;
	z3=cloud->points[*itr].z;
	
	float a= (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
	float b= (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
	float c= (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
	float d= -(a*x1+b*y1+c*z1);	

    for(int index=0;index<cloud->points.size(); index++)
    {
	
		if (inliers.count(index)>0)
			continue;
		
		//pcl::PointXYZ point=cloud->points[index];
        PointT point=cloud->points[index];
		float x4=point.x;
		float y4=point.y;
		float z4=point.z;

        float dis =fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

		if(dis <= distanceThreshold)
			inliers.insert(index);
    }
	if(inliers.size()>inliersResult.size())
		inliersResult=inliers;
    }
    
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index=0; index<cloud->points.size();index++)
    {
        //pcl::PointXYZ point=cloud->points[index];
        PointT point=cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else       
            cloudOutliers->points.push_back(point);
    }          
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);
    return segResult;
}


template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(int index: getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);
        cloudCluster->width=cloudCluster->points.size();
        cloudCluster->height=1;
        cloudCluster->is_dense=true;
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>& cluster,std::vector<bool>& processed,KdTree* tree, float distanceTol)
{
	processed[idx]=true;
	cluster.push_back(idx);
	std::vector<int> nearest=tree->search(cloud->points[idx],distanceTol);

	for(int id:nearest)
	{
		if(!processed[id])
			clusterHelper(id,cloud,cluster,processed,tree,distanceTol);
	}
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>:: euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol,int minSize,int maxSize)
{

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool>processed(cloud->points.size(),false);

    for(size_t idx=0;idx<cloud->points.size();++idx)
    {
        if(processed[idx]==false)
        {
            std::vector<int>cluster_idx;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
            clusterHelper(idx,cloud,cluster_idx,processed,tree,distanceTol);

            if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize)
            {
                    for(int i=0;i< cluster_idx.size();i++)
                    {
                        PointT point;
                        point = cloud->points[cluster_idx[i]];
                        cloudCluster->points.push_back(point);
                    }
                    cloudCluster->width=cloudCluster->points.size();
                    cloudCluster->height=1;
                    clusters.push_back(cloudCluster);
            }
            else
            {
                for(int i;i<cluster_idx.size();i++)
                {
                    processed[cluster_idx[i]]=false;
                }
            }
            
        }
    }
	
	return clusters;

}

template <typename PointT>
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


template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template <typename PointT>
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


template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}	
	

