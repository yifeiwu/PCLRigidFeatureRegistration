# PCLRigidFeatureRegistration
Use PCL to rigidly register two point clouds using feature based sample consensus alignment

This demonstrates using the Point Cloud Library to align two point clouds using point based descriptors. Two PCD (point cloud files) are submitted as inputs. The normal vectors for the clouds will be estimated and fast point feature histogram descriptors are calculated. The radius (feature size) can be adjusted as necessary. A rigid alignment is calculated using sample consensus. 

http://pointclouds.org/documentation/tutorials/fpfh_estimation.php
