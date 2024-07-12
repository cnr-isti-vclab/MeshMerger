

#include"MeshVolume.h"

using namespace charity;
using namespace vdb;


cMeshVolume::cMeshVolume()
{
	gridDimension = 128;
	narrowbandWidth = 3;
	center = openvdb::Vec3d(-INFINITY, -INFINITY, -INFINITY);
	extent = openvdb::Vec3d(0, 0, 0);
	voxelSize = 0.0;
	volumeSize = openvdb::Vec3d(0, 0, 0);
	v_Flags = MeshVolumeFlags();
	exbw = 0.0f;
	inbw = 0.0f;
	isoValue = 0.0f,
	adapt = 0.0f;

}
cMeshVolume::cMeshVolume(const mesh& alignedMesh, float isValue, float adaptive, int dim, int bw, std::string outFile)
{
	openvdb::initialize();
	gridDimension = dim;
	narrowbandWidth = bw;
	center = openvdb::Vec3d(-INFINITY, -INFINITY, -INFINITY);
	extent = openvdb::Vec3d(0, 0, 0);
	voxelSize = 0.0;
	volumeSize = openvdb::Vec3d(0, 0, 0);
	v_Flags = MeshVolumeFlags();
	exbw = 0.0f;
	inbw = 0.0f;
	isoValue =isValue,
    adapt = adaptive;
	outFileName = outFile;


	// tranfer vertex data
	cloudNormPtr pn_temp(new pnc);
	pcl::fromPCLPointCloud2(alignedMesh.cloud, *pn_temp);
	for(int i = 0; i < pn_temp->size(); i++)
	{
		openvdb::Vec3s v = openvdb::Vec3s(pn_temp->points[i].x, pn_temp->points[i].y, pn_temp->points[i].z);
		volMesh.bbox.expand(v);
		volMesh.points.push_back(v);
	}
	// transfer face data
	for (const auto& face : alignedMesh.polygons)
	{
		volMesh.faces.push_back(openvdb::Vec3I(face.vertices[0], face.vertices[1], face.vertices[2]));
	}
}
void cMeshVolume::dataConversion(const mesh& alignedMesh)
{
	// tranfer vertex data
	cloudNormPtr pn_temp(new pnc);
	pcl::fromPCLPointCloud2(alignedMesh.cloud, *pn_temp);
	for(int i = 0; i < pn_temp->size(); i++)
	{
		openvdb::Vec3s v = openvdb::Vec3s(pn_temp->points[i].x, pn_temp->points[i].y, pn_temp->points[i].z);
		volMesh.bbox.expand(v);
		volMesh.points.push_back(v);
	}
	// transfer face data
	for (const auto& face : alignedMesh.polygons)
	{
		volMesh.faces.push_back(openvdb::Vec3I(face.vertices[0], face.vertices[1], face.vertices[2]));
	}
}
void cMeshVolume::computeParams()
{
	// Set volume size and voxel size
	float expand = 1.0;
	center = volMesh.bbox.getCenter();
	extent = volMesh.bbox.extents() * expand;

	float length = openvdb::math::Max(
		extent.x(),
		extent.y(),
		extent.z());

	voxelSize = length / (gridDimension - 1);
	std::cout << "voxel size:" << voxelSize << std::endl;
	if (voxelSize < 1e-5)
	{
		std::cout << "The voxel size (" << voxelSize << ") is too small.";
		exit(1);
	}
	openvdb::Vec3d volume_size;
	if (v_Flags.flag_cube)
	{
		volume_size.init(length, length, length);
	}
	else
	{
		volume_size = extent;
	}

	// Set narrow-band width
	exbw = inbw = narrowbandWidth;
	if (v_Flags.flag_full)
	{
		exbw = gridDimension;
	}

	if (v_Flags.flag_world)
	{
		exbw = exbw / voxelSize;
		inbw = inbw / voxelSize;
	}

	if (v_Flags.flag_fill || v_Flags.flag_full)
	{
		inbw = std::numeric_limits<float>::max();
	}

	transform = openvdb::math::Transform::createLinearTransform(voxelSize);
	if (v_Flags.flag_cell)
	{
		transform->postTranslate({ voxelSize / 2, voxelSize / 2, voxelSize / 2 });
	}
}
void cMeshVolume::buildGrid()
{
	if (v_Flags.flag_unsigned)
	{
		grid = openvdb::tools::meshToUnsignedDistanceField<openvdb::FloatGrid>(*transform, volMesh.points, volMesh.faces, std::vector<openvdb::Vec4I>(), exbw);
		// grid = openvdb::tools::topologyToLevelSet<openvdb::FloatGrid>(*grid, bw);

	}
	else
	{
		grid = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(*transform, volMesh.points, volMesh.faces, std::vector<openvdb::Vec4I>(), exbw, inbw);
		//grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(*transform, mesh.points, mesh.faces, std::vector<openvdb::Vec4I>(), bw);
	}
}
void cMeshVolume::convertUsdfToMesh(bool flagRelax, bool write)
{
	//vMesh vMesh_temp;
	openvdb::tools::volumeToMesh(*grid, outVolMesh.points, outVolMesh.faces, outVolMesh.quads, isoValue, adapt, flagRelax);

	if (write)
	{
		charity::geometry::writePLY(outFileName, outVolMesh.points, outVolMesh.faces, outVolMesh.quads);
		//std::cout << "No. of faces:" << outVolMesh.faces.size() + outVolMesh.quads.size() << std::endl;
	}


	
}