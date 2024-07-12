
#pragma once


#define WIN32_LEAN_AND_MEAN	
//#include "targetver.h"
#include <omp.h>
#include <stdio.h>
#ifdef WIN32
#include <tchar.h>
#endif
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <boost/format.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>   
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_rejection_one_to_one.h>
//#include <pcl/registration/correspondence_rejection_features.h> 
#include <pcl/registration/correspondence_rejection_sample_consensus.h> 
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/geometry/polygon_mesh.h>

#include<pcl/PolygonMesh.h>
#include <pcl/geometry/triangle_mesh.h>
#include<pcl/geometry/mesh_io.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl/io/obj_io.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"



//s#include <pcl/visualization/cloud_viewer.h>



