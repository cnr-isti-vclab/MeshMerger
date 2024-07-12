


#include"CRegistration.h"
#include"TRO_Utilities.h"
#include"RegBase.h"

#include"teaser/utility.h"
#include"teaser/CTeaser.h"
#include"../IO/json_reader.h"


using namespace charity;
using namespace Registration;

//template<typename K, typename T>
Eigen::Matrix3Xf ToEigenMatrix(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud_in)
{
	Eigen::Matrix3Xf out_matrix = Eigen::Matrix3Xf::Zero(3, cloud_in->points.size());

	for (size_t idx = 0; idx != cloud_in->size(); ++idx) {
		out_matrix.col(idx) = cloud_in->points[idx].getVector3fMap();
		//std::cout << out_matrix.col(idx).transpose() << std::endl;
	}
	return out_matrix;
}


//iRegistration::iRegistration()
//{
//
//}
iRegistration::iRegistration(algo alg, /*double sampleRadius, double featureRadius,*/ double samplRadMul, double featRadMul, int nThreads)
{
	m_algo = alg;
	/*m_samplRad = sampleRadius;
	m_featRad = featureRadius;*/
	m_sampMul = samplRadMul;
	m_featMul = featRadMul;
	m_nThreads = nThreads;
	m_nTotalView = 0;

}

mat4f iRegistration::getTransformation(int vpPairIdx)
{

	if (vpPairIdx > 0)
		return m_TransformationArray[vpPairIdx];

	return  mat4f::Identity();

}
void iRegistration::setViewPairForRegistration(const mesh& polygonMeshA, const mesh& polygonMeshB)
{
	m_viewPairs.push_back(std::make_pair(polygonMeshA, polygonMeshB));
	//increase view everytime anew pair is pushed
	m_ldstats.push_back(loadStatus(false, false));
	m_nTotalView += 1;
}

void iRegistration::AlignMesh()
{

	// create a vector of std::shared_ptr
	//std::vector< std::shared_ptr<CTEASER>>regis()
	std::shared_ptr<CTEASER> pTeaser(new CTEASER());
	string tmp("Teaser");
	pTeaser->setParams(tmp);

	// compute meshRes: considering all the pts are sampled from the same sensor
	// the target mesh of 1st pair is considered
	cloudNormPtr pn_temp(new pnc);
	pcl::fromPCLPointCloud2(m_viewPairs[0].second.cloud, *pn_temp);

	//// set the sampling and feature radius. 
	double dRes = computeModelResFun(pn_temp);
	std::cout << "avg-pt-dist(resl):" << dRes << std::endl;
	int cnt = 0;
	bool meshConcat = true;
	for (auto const& mv : m_viewPairs)
	{
		// making it eqvlnt to python impl
		//dRes = 2.8 * dRes;

		//set threads
		pTeaser->setNumThreads(m_nThreads);
		// first param is voxel sampling radius and the second is feature radius
		//pTeaser->setRadius(m_sampMul * dRes, m_featMul * dRes);
		pTeaser->setRadius(0.04, 0.04 * 5.0);

		//// set input cloud.
		cloudNormPtr pn_temp_src(new pnc);
		/*if (!m_ldstats[cnt].loadSrc)*/
		pcl::fromPCLPointCloud2(mv.first.cloud, *pn_temp_src);
		pTeaser->setInputSource(pn_temp_src);

		cloudNormPtr pn_temp_tgt(new pnc);
		pcl::fromPCLPointCloud2(mv.second.cloud, *pn_temp_tgt);

		pTeaser->setInputTarget(pn_temp_tgt);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointNormal>());
		Eigen::Matrix4f Tf0 = Eigen::Matrix4f::Identity();

		auto t5 = std::chrono::steady_clock::now();

		pTeaser->align(*cloud_aligned);

		auto t6 = std::chrono::steady_clock::now();
		std::cout << "Time for alignment: " << std::chrono::duration<double>(t6 - t5).count() << "s\n\n";
		Tf0 = pTeaser->getFinalTransformation();
		//Eigen::Matrix4f icp_align = fineAlignment(pn_temp_src, pn_temp_tgt, 0.5 * dRes, Tf0);
		std::cout << Tf0 << std::endl;
		m_TransformationArray.push_back(Tf0);
		//std::cout << icp_align *Tf0 << std::endl;
		if (meshConcat)
		{
			m_FinalMesh = io::transformMesh(mv.first, Tf0);
			meshConcat = false;
			m_FinalMesh = m_FinalMesh + mv.second;
			continue;
		}
		m_FinalMesh = m_FinalMesh + io::transformMesh(mv.first, Tf0);

		// concatenate mesh
	}
}

//Eigen::Matrix4f iRegistration::fineAlignment(const cloudNormPtr& ptA, const cloudNormPtr& ptB, double samplRad, Eigen::Matrix4f init_tf)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>());
//	//pcl::PointCloud<pcl::Normal>::Ptr normalsA(new pcl::PointCloud<pcl::Normal>());
//	decoupleCloud(ptA, cloudA/*, normalsA*/);
//	voxelSampleInPlaceFun(cloudA, samplRad);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::Normal>::Ptr normalsB(new pcl::PointCloud<pcl::Normal>());
//	decoupleCloud(ptB, cloudB, normalsB);
//	SICP::Parameters params;
//	Eigen::Affine3f coarsemat(init_tf);
//	params.initialTransform = &coarsemat;
//	params.max_icp = 10;
//
//	Eigen::Matrix3Xf A = ToEigenMatrix(cloudA);
//	Eigen::Matrix3Xf B = ToEigenMatrix(cloudB);
//
//	Eigen::Affine3f fine_tf = SICP::point_to_point(A, B,params);
//	return fine_tf.matrix();
//}

