
#include"TRO_Utilities.h"

void load_bunny(string& sFile_src, string& sFile_tgt) {
	string sDataRoot = "example_data\\charity\\";
	sFile_src = sDataRoot + "density_low_left_all.ply";
	sFile_tgt = sDataRoot + "density_low_middle_all.ply";
}

//// compute the spatial resolution of point cloud. 
double computeModelResFun(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {
	pcl::KdTreeFLANN<pcl::PointNormal>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointNormal>());
	kd_tree->setInputCloud(cloud);
	double dDist = 0.0;
	for (int i = 0; i < cloud->points.size(); i++) {
		pcl::PointNormal pt = cloud->points[i];
		vector<int> vIdx;
		vector<float> vDist_sq;
		kd_tree->nearestKSearch(pt, 2, vIdx, vDist_sq);
		dDist += sqrt(vDist_sq[1]);
	}
	double dRes = dDist / cloud->points.size();
	return dRes;
}

Eigen::Matrix4f printTransformation(const Eigen::Matrix4f& min_tf)
{
	std::cout << min_tf(0, 0) << " " << min_tf(0, 1) << " " << min_tf(0, 2) << " " << min_tf(0, 3) << " " <<
		min_tf(1, 0) << " " << min_tf(1, 1) << " " << min_tf(1, 2) << " " << min_tf(1, 3) << " " <<
		min_tf(2, 0) << " " << min_tf(2, 1) << " " << min_tf(2, 2) << " " << min_tf(2, 3) << " " <<
		min_tf(3, 0) << " " << min_tf(3, 1) << " " << min_tf(3, 2) << " " << min_tf(3, 3) << std::endl;
	return min_tf;
}

Eigen::Matrix4f Rotate(float rotinDegree, int axis)
{
	float rotinRad = rotinDegree * (M_PI / 180.0);
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	if (axis == 0)
	{
		// The same rotation matrix as before; theta radians around Z axis
		transform_2.rotate(Eigen::AngleAxisf(rotinRad, Eigen::Vector3f::UnitX()));

	}
	if (axis == 1)
	{
		// The same rotation matrix as before; theta radians around Z axis
		transform_2.rotate(Eigen::AngleAxisf(rotinRad, Eigen::Vector3f::UnitY()));
	}
	if (axis == 2)
	{
		// The same rotation matrix as before; theta radians around Z axis
		transform_2.rotate(Eigen::AngleAxisf(rotinRad, Eigen::Vector3f::UnitZ()));
	}
	return transform_2.matrix();
}
Eigen::Matrix4f changeSymmetry(int axis)
{
	Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
	if (axis == 0)
	{
		transf(0, 0) = -1;
	}
	if (axis == 1)
	{
		transf(1, 1) = -1;
	}
	if (axis == 2)
	{
		transf(2, 2) = -1;
	}
	return transf;
}
std::vector<float> computeViewPosDifference(const std::vector<Eigen::Vector3f>& devicePosOrient)
{
	float norm1 = (devicePosOrient[0] - devicePosOrient[2]).norm();
	float norm2 = (devicePosOrient[2] - devicePosOrient[4]).norm();
	float norm3 = (devicePosOrient[0] - devicePosOrient[4]).norm();
	std::vector<float>normVal;
	normVal.push_back(norm1);
	normVal.push_back(norm2);
	normVal.push_back(norm3);
	return normVal;
}

Eigen::Vector3f vectorFromAngle(float radAngle, int dir)
{
	Eigen::AngleAxisf rotVec = Eigen::AngleAxisf::Identity();
	if (dir == 2)
		rotVec = Eigen::AngleAxisf(radAngle, Eigen::Vector3f::UnitZ());
	else if(dir == 1)
		rotVec = Eigen::AngleAxisf(radAngle, Eigen::Vector3f::UnitY());
	else if( dir == 0)
		rotVec = Eigen::AngleAxisf(radAngle, Eigen::Vector3f::UnitX());
	Eigen::Vector3f vec(rotVec.angle()* rotVec.axis());

	return vec;

}

float directionOffset(Eigen::Vector3f norm, Eigen::Vector3f dir)
{
	float val = norm.dot(dir);
	return val;
}

void computeDirectionOffset(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals, Eigen::Vector3f dir)
{
	for (const auto& elem : cloud_normals->points)
	{
		Eigen::Vector3f norm = elem.getNormalVector3fMap();
		float val = directionOffset(norm, dir);
	}
}

Eigen::Matrix4f constructLocalFrame(const Eigen::Vector3f dir, const Eigen::Vector3f pos)
{
// dir vec is pointing to -z axis
	Eigen::Matrix4f locFrame = Eigen::Matrix4f::Identity();
	Eigen::Vector4f zVec = Eigen::Vector4f(-dir(0), -dir(1), -dir(2), 0);
	Eigen::Vector4f yVec = Eigen::Vector4f(0, 1, 0, 0);
	Eigen::Vector4f xVec = Eigen::Vector4f::Zero();
	xVec.head<3>() = (yVec.head<3>().cross(zVec.head<3>())).normalized();
	//
	locFrame.col(0) = xVec;
	locFrame.col(1) = yVec;
	locFrame.col(2) = zVec;
	locFrame.col(3).head<3>() = pos;
	std::cout << "local frame:\n" << locFrame << std::endl;

	return locFrame.inverse();
	
}