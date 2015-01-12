//------------------------------------------------------------------------------------ 
//Based off code from http://www.pcl-users.org/Very-poor-registration-results-td3569265.html
#include <iostream>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_cloud.h" 
#include "pcl/kdtree/kdtree_flann.h" 
#include "pcl/filters/passthrough.h" 
#include "pcl/filters/voxel_grid.h" 
#include "pcl/features/fpfh.h" 
//------------------------------------------------ 

using namespace pcl; 
using namespace std; 


const double FILTER_LIMIT = 1000.0; 
const int MAX_SACIA_ITERATIONS = 2000; 

//units are meters: 
const float VOXEL_GRID_SIZE = 0.03; 
const double NORMALS_RADIUS = 20; 
const double FEATURES_RADIUS = 50; 
const double SAC_MAX_CORRESPONDENCE_DIST =2000; 
const double SAC_MIN_CORRESPONDENCE_DIST =3;

void filterCloud( PointCloud<PointXYZ>::Ptr ); 
PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZ>::Ptr incloud ); 
PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloud<PointXYZ>::Ptr incloud, PointCloud<Normal>::Ptr normals ); 
void view( PointCloud<pcl::PointXYZ> & cloud ); 
SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>
	align( PointCloud<PointXYZ>::Ptr c1, PointCloud<PointXYZ>::Ptr c2, 
		PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2 ); 

int main(int argc, char** argv) 
{ 
		time_t starttime = time(NULL); 

		cout << "Loading clouds..."; 
		cout.flush(); 

		//open the clouds 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);     
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);     

		pcl::io::loadPCDFile("cornercloud1", *cloud1); 
		pcl::io::loadPCDFile("cornercloud2", *cloud2); 
 

//        downsample the clouds, but store the downsampled clouds seperately 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1ds (new pcl::PointCloud<pcl::PointXYZ>);     
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2ds (new pcl::PointCloud<pcl::PointXYZ>);     
		VoxelGrid<PointXYZ> vox_grid; 
		vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE ); 
		vox_grid.setInputCloud( cloud1 ); 
		vox_grid.filter( *cloud1ds ); 

        vox_grid.setInputCloud( cloud2 ); 
        vox_grid.filter( *cloud2ds ); 

		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nCalculating normals..."; 
		cout.flush();     

        //compute normals 
        pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals( cloud1ds ); 
        pcl::PointCloud<pcl::Normal>::Ptr normals2 = getNormals( cloud2ds ); 

		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nComputing local features..."; 
		cout.flush(); 

        //compute local features 
        PointCloud<FPFHSignature33>::Ptr features1 = getFeatures( cloud1ds, normals1 ); 
        PointCloud<FPFHSignature33>::Ptr features2 = getFeatures( cloud2ds, normals2 ); 
        
		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nComputing initial alignment using SAC..."; 
		cout.flush(); 

        //Get an initial estimate for the transformation using SAC 
        //returns the transformation for cloud2 so that it is aligned with cloud1 
        SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia = align( cloud1ds, cloud2ds, features1, features2 ); 
        Eigen::Matrix4f	init_transform = sac_ia.getFinalTransformation(); 
		cout << init_transform;
        transformPointCloud( *cloud2, *cloud2, init_transform ); 
        pcl::PointCloud<pcl::PointXYZ> final = *cloud1; 
        final += *cloud2; 

		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\n"; 
		cout << "Opening aligned cloud; will return when viewer window is closed."; 
		cout.flush(); 

		view(final); 
		  
		pcl::io::savePCDFile ("test.pcd", *features1);//this output is 33 values per line(point)
		return 1; 

} 

//computes the transformation for cloud2 so that it is transformed so that it is aligned with cloud1 
SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>
		align( PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2, 
			PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2 ) { 

		SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia; 
		Eigen::Matrix4f final_transformation;	
		sac_ia.setInputCloud( cloud2 ); 
		sac_ia.setSourceFeatures( features2 ); 
		sac_ia.setInputTarget( cloud1 ); 
		sac_ia.setTargetFeatures( features1 ); 
		sac_ia.setMaximumIterations( MAX_SACIA_ITERATIONS ); 
		sac_ia.setMinSampleDistance (SAC_MIN_CORRESPONDENCE_DIST);
		sac_ia.setMaxCorrespondenceDistance (SAC_MAX_CORRESPONDENCE_DIST);
		PointCloud<PointXYZ> finalcloud;	
		sac_ia.align( finalcloud ); 
		sac_ia.getCorrespondence();
		return sac_ia; 
} 

PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr normals ) { 

		PointCloud<FPFHSignature33>::Ptr features = PointCloud<FPFHSignature33>::Ptr (new PointCloud<FPFHSignature33>); 
		search::KdTree<PointXYZ>::Ptr search_method_ptr = search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>); 
		FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est; 
		fpfh_est.setInputCloud( cloud ); 
		fpfh_est.setInputNormals( normals ); 
		fpfh_est.setSearchMethod( search_method_ptr ); 
		fpfh_est.setRadiusSearch( FEATURES_RADIUS ); 
		fpfh_est.compute( *features ); 
        return features; 
} 

PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZ>::Ptr incloud ) { 

		PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr (new PointCloud<Normal>); 
		NormalEstimation<PointXYZ, Normal> norm_est; 
		norm_est.setInputCloud( incloud ); 
		norm_est.setRadiusSearch( NORMALS_RADIUS ); 
		norm_est.compute( *normalsPtr ); 
		return normalsPtr; 
} 

void filterCloud( PointCloud<PointXYZ>::Ptr pc ) { 

		pcl::PassThrough<pcl::PointXYZ> pass; 
		pass.setInputCloud(pc); 
		pass.setFilterFieldName("x"); 
		pass.setFilterLimits(0, FILTER_LIMIT); 
		pass.setFilterFieldName("y"); 
		pass.setFilterLimits(0, FILTER_LIMIT); 
		pass.setFilterFieldName("z"); 
		pass.setFilterLimits(0, FILTER_LIMIT); 
		pass.filter(*pc);   

} 

void view( PointCloud<pcl::PointXYZ> & cloud ) { 

        pcl::visualization::CloudViewer viewer1("Cloud Viewer"); 
        viewer1.showCloud( cloud.makeShared() );     
		while( !viewer1.wasStopped() ); 
		return; 
} 