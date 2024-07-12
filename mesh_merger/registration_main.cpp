
#include<iostream>
#include<chrono>
#include<omp.h>
#include<string>

#include"Registration/TRO_Utilities.h"
#include"Registration/RegBase.h"
#include"teaser/utility.h"
#include"teaser/CTeaser.h"
#include"Registration/prjheader.h"
#include"IO/json_reader.h"
#include"Registration/CRegistration.h"
#include"MeshLevelSet/MeshVolume.h"


void writePLYMeshBinary(const mesh& m, const string fileName)
{
	int size = m.cloud.width * m.cloud.height;
	std::cout << "size of cloud:" << size << std::endl;
	ofstream os(fileName, ios::binary);
	if (!os.is_open()) {
		throw invalid_argument("Error writing to ply file \"" + fileName + "\"");
	}

	// Write vertices
	os << "ply\n";
	os << "format binary_little_endian 1.0\n";
	os << "element vertex " << size << endl;
	os << "property float x\n";
	os << "property float y\n";
	os << "property float z\n";
	os << "property float nx\n";
	os << "property float ny\n";
	os << "property float nz\n";
	os << "element face " << m.polygons.size() << endl;
	os << "property list uchar int vertex_index\n";
	os << "end_header\n";

	// Write vertex data
	 // Write vertex data
	const unsigned char* data_ptr = m.cloud.data.data(); // Start of the point cloud data
	const size_t point_step = m.cloud.point_step;
	 // Write vertex data
	for (size_t i = 0; i < size; ++i)
	{
		float x = *reinterpret_cast<const float*>(data_ptr + m.cloud.fields[0].offset);
		float y = *reinterpret_cast<const float*>(data_ptr + m.cloud.fields[1].offset);
		float z = *reinterpret_cast<const float*>(data_ptr + m.cloud.fields[2].offset);
		float nx = *reinterpret_cast<const float*>(data_ptr + m.cloud.fields[3].offset);
		float ny = *reinterpret_cast<const float*>(data_ptr + m.cloud.fields[4].offset);
		float nz = *reinterpret_cast<const float*>(data_ptr + m.cloud.fields[5].offset);

		os.write(reinterpret_cast<char*>(&x), sizeof(float));
		os.write(reinterpret_cast<char*>(&y), sizeof(float));
		os.write(reinterpret_cast<char*>(&z), sizeof(float));
		os.write(reinterpret_cast<char*>(&nx), sizeof(float));
		os.write(reinterpret_cast<char*>(&ny), sizeof(float));
		os.write(reinterpret_cast<char*>(&nz), sizeof(float));
		// Move to the next point in the point cloud data
		data_ptr += point_step;
	}

	// Write faces
	for (const auto& polygon : m.polygons) {
		char numVertices = 3; // Assuming triangular faces
		os.write(reinterpret_cast<const char*>(&numVertices), sizeof(char));
		os.write(reinterpret_cast<const char*>(&polygon.vertices[0]), sizeof(uint32_t));
		os.write(reinterpret_cast<const char*>(&polygon.vertices[1]), sizeof(uint32_t));
		os.write(reinterpret_cast<const char*>(&polygon.vertices[2]), sizeof(uint32_t));
	}
}

int main(int argc, char* argv[])
{
	auto t1 = std::chrono::steady_clock::now();

	// read config file
	std::vector<std::pair<string, string>>  configData = io::readConfigFile("config.txt");

	// Check if three file paths are provided as command line arguments
	for (int i = 0; i < argc; ++i) {
		std::cout << "Argument " << i << ": " << argv[i] << std::endl;
	}
	if (argc != 6) {

		std::cerr << "Usage: " << argv[0] << " <file_path_1> <file_path_2> <file_path_3>" << std::endl;
		return -1;
	}

	// Get the file paths from the command line arguments
	std::string fpo = argv[1];
	bool fliporient = false;
	if (fpo == "true")
		fliporient = true;

	std::string filePath1 = argv[2];
	std::string filePath2 = argv[3];
	//std::string filePath3 = argv[3];
	mesh m1;
	if (pcl::io::loadPLYFile(filePath1, m1) < 0) // "D:/CHARITY/code/node_js_scripts/meshData_mesh1.ply"
	{
		std::string base_filename = filePath1.substr(filePath1.find_last_of("/") + 1);
		std::cout << "error-loading file" << base_filename << std::endl;
		return -1;
	}
	
	// apply symmetry & rotation
	Eigen::Matrix4f symm = changeSymmetry(1);
	// chnage orientation of mesh
	Eigen::Matrix4f rotMatrix = Rotate(180, 2);
	Eigen::Matrix4f ft = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f rotY = Rotate(180, 1);
	//invZ(2, 2) = -1;
	ft = rotY * rotMatrix * symm;
	
	m1 = io::transformMesh(m1, ft);
	m1 = io::invertFaceOrientation(m1);
	
	mesh m2;

	if (pcl::io::loadPLYFile(filePath2, m2) < 0) // "D:/CHARITY/code/node_js_scripts/meshData_mesh1.ply"
	{
		std::string base_filename = filePath2.substr(filePath2.find_last_of("/") + 1);
		std::cout << "error-loading file" << base_filename << std::endl;
		return -1;
	}
	std::cout << "m2 cloud field size:" << m2.cloud.fields.size() << std::endl;
	//reader.concatenateMesh();
	//std::string fileNameB = f3;//jsonfile2.substr(jsonfile2.find_last_of("/") + 1, jsonfile2.length());
	if (!fliporient)
	{
		ft = Eigen::Matrix4f::Identity();
		m2 = io::transformMesh(m2, ft);
	}
	else
	{
		m2 = io::transformMesh(m2, ft);
		m2 = io::invertFaceOrientation(m2);
	}
	
	// Instantiate Registration class
	charity::Registration::iRegistration regis(static_cast<charity::algo>(1), stof(configData[0].second), stof(configData[1].second), stoi(configData[5].second));  //2.8, 14.0 1.45, 7.3
	regis.setViewPairForRegistration(m1, m2);
	//regis.setViewPairForRegistration(m3, m2);
	//auto t5 = std::chrono::steady_clock::now();
	regis.AlignMesh();

	mesh output_mesh = regis.getFinalMesh();


	if (stoi(configData[6].second))
	{
		std::string regis_fileName = argv[4];
		//writePLYMeshBinary(output_mesh, regis_fileName);
		pcl::io::savePLYFileBinary(regis_fileName, output_mesh);
		std::cout << "registered-output-write complete" << std::endl;
	}

	// mesh-volume conversion
	//auto t3 = std::chrono::steady_clock::now();
	int gridD = stoi(configData[2].second); //256;
	float iso = stof(configData[3].second); //0.02;
	float adap = stof(configData[4].second); //0.1;

	std::stringstream ss;
	ss << std::fixed << std::setprecision(3) << iso;

	//std::string outVolFileName = configData[12].second/*"D:/CHARITY/Data/combo_mesh_"*/ + configData[13].second + "_"
	//	+ ss.str() + "_" + std::to_string(gridD) + ".ply";
	std::string outVolFileName = argv[5];

	charity::vdb::cMeshVolume cmv(output_mesh, iso, adap, gridD, 3, outVolFileName);
	cmv.computeParams();
	auto t3 = std::chrono::steady_clock::now();
	cmv.buildGrid();
	auto t4 = std::chrono::steady_clock::now();
	std::cout << "Time for volume comp.: " << std::chrono::duration<double>(t4 - t3).count() << "s\n\n";
	cmv.convertUsdfToMesh(true, stoi(configData[7].second));

	auto t2 = std::chrono::steady_clock::now();
	std::cout << "Time for complete process: " << std::chrono::duration<double>(t2 - t1).count() << "s\n\n";
	//system("pause");
	//cin.get();
	return 0;
}