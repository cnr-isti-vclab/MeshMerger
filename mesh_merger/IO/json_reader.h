#pragma once

#include<string>
#include<vector>
#include <sstream>  // to store the contents from the file
#include <fstream>  // to open/read the file
#include<exception>
#include<filesystem>

// including JsonCpp header file
//#include <json/json.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include"../Registration/prjheader.h"
#include"defines.h"

using pc2 = pcl::PCLPointCloud2::Ptr;
using rs = rapidjson::SizeType;

#ifdef _MSC_VER
#undef GetObject
#endif

namespace io
{

	mesh transformMesh(const mesh& m, const Eigen::Matrix4f& tf);
	vecPair readConfigFile(std::string configFile);
	std::string getdownloadedFile(std::string downloadurl);
	mesh invertFaceOrientation(const mesh& m);
	bool existsFile(std::string fileName);
	std::string loadFileFromPath(std::string url);
	std::string getFileNameFromPath(std::string url);
}
