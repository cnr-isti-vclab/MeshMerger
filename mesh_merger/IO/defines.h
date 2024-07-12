#pragma once

#include<Eigen/Core>
#include"../Registration/prjheader.h"

//#define LOG_OUTPUT
//#define WRITE_FILE
#define LOG 

using mat4f = Eigen::Matrix4f;
using mesh = pcl::PolygonMesh;
using meshVector = std::vector<mesh>;
using face = pcl::Vertices;
using facelist = std::vector<face>;
using pn = pcl::PointNormal;
using pnc = pcl::PointCloud<pn>;
using cloudNormPtr = pnc::Ptr;
using vecPair = std::vector< std::pair<std::string, std::string>>;
