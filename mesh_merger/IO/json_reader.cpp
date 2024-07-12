#include <iostream>
#include <fstream>
#include<chrono>



#include"json_reader.h"
//#include"datadownload.h"


using namespace std;
using namespace io;
using namespace rapidjson;


mesh io::transformMesh(const mesh& m, const Eigen::Matrix4f& tf)
{
	mesh mesh_temp = m;
	pnc pn_temp;
	pcl::fromPCLPointCloud2(mesh_temp.cloud, pn_temp);
	pcl::transformPointCloudWithNormals(pn_temp, pn_temp, tf);
	pcl::toPCLPointCloud2(pn_temp, mesh_temp.cloud);
	//mesh m_rev = invertFaceOrientation(mesh_temp);
	return mesh_temp;
}

mesh io::invertFaceOrientation(const mesh& m)
{
	mesh mesh_temp;
	pcl::copyPointCloud(m.cloud, mesh_temp.cloud);
	// change vertex idx order for face
	mesh_temp.polygons.resize(m.polygons.size());
	
	for (size_t i = 0; i < m.polygons.size(); i++)
	{
		face f;
		f.vertices.resize(3);
		//std::cout << m.polygons[i].vertices[0] << "," << m.polygons[i].vertices[1] << "," << m.polygons[i].vertices[2] << std::endl;
		//std::reverse_copy(m.polygons[i].vertices.begin(), m.polygons[i].vertices.end(), f.vertices.begin());
		f.vertices[0] = m.polygons[i].vertices[1];
		f.vertices[1] = m.polygons[i].vertices[0];
		f.vertices[2] = m.polygons[i].vertices[2];
		//std::cout << f.vertices[0] << "," <<f.vertices[1] << "," << f.vertices[2] << std::endl;
		mesh_temp.polygons[i] = f;
	}
	return mesh_temp;
}

vecPair io::readConfigFile(std::string configFile) {

	std::ifstream cFile(configFile);
	vecPair configData;
	if (cFile.is_open())
	{
		std::string line;
		while (std::getline(cFile, line)) {
			//			line.erase(std::remove_if(line.begin(), line.end(), isspace),line.end());
			if (line[0] == '#' || line.empty())
				continue;
			auto delimiterPos = line.find("=");
			std::string name = line.substr(0, delimiterPos);
			std::string value = line.substr(delimiterPos + 1);
			std::cout << name << "--> " << value << '\n';
			configData.push_back(std::make_pair(name, value));
		}

	}
	else {
		std::cerr << "Couldn't open config file for reading.\n";
	}

	return configData;

}
std::string io::getdownloadedFile(std::string downloadurl)
{
	// code removed
	return downloadurl;
}
bool io:: existsFile(std::string fileName)
{
	const std::filesystem::path p = fileName;
	return std::filesystem::exists(p);
}

std::string io::loadFileFromPath(std::string url)
{
	std::string fileName = getFileNameFromPath(url);

	bool existStat = existsFile(fileName);
	if (existStat)
		return fileName;

	std::string f1 = io::getdownloadedFile(url);
	return f1;

}

std::string io:: getFileNameFromPath(std::string url)
{
	std::string f = url.substr(url.find_last_of("/") + 1, url.length());
	return f;
}