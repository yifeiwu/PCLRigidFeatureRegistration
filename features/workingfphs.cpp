#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <iostream>


#include<vtkPolyDataReader.h>
#include<vtkSmartPointer.h>
#include<vtkPolyData.h>
#include<vtkPoints.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;






int main (int argc, char** argv)
{
std::string inputfile=argv[1];
std::string outfile=argv[2];


 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (inputfile.c_str(), *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::cout << "Loaded ";
  




  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (20);

  // Compute the features
  ne.compute (*cloud_normals);








  //pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());


  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud);
  fpfh.setInputNormals (cloud_normals);
  // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointXYZ>::Ptr tree2 (new pcl::search::KdTree<PointXYZ>);

  fpfh.setSearchMethod (tree2);

  // Output datasets
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (25);

  // Compute the features
  fpfh.compute (*fpfhs);

  // fpfhs->points.size () should have the same size as the input cloud->points.size ()*



 





print_info ("done");


//pcl::PCDWriter w;
//w.writeASCII<pcl::PointXYZ> (outfile.c_str(), *cloud_temp);
//pcl::io::savePCDFileASCII(argv[1], *cloud_temp);

savePCDFile("fpfhs.pcd",*fpfhs);

  return (0);
}
