#pragma once

#include<iostream>

#pragma once

#include<vector>
#include<algorithm>
#include<Eigen/Core>

#include"../IO/defines.h"
#include"../Registration/prjheader.h"
//#include"sparse_icp.h"
#include"teaser/registration.h"

using namespace teaser;


namespace charity
{
	enum class algo
	{
		teaser = 0,
		fgr

	};

    struct loadStatus
	{
		bool loadSrc;
		bool loadTgt;
		loadStatus(bool s, bool t):loadSrc(s), loadTgt(t){}
	};
	namespace Registration
	{
		class iRegistration
		{
		public:
			mat4f getTransformation(int vpPairIdx);
			iRegistration() = default;
			iRegistration(algo alg, double sampleRadius, /*double featureRadius, double samplRadMul, */double featRadMul, int nThreads);
			void setViewPairForRegistration(const mesh& polygonMeshA, const mesh& polygonMeshB);
			mesh getFinalMesh() {
				return m_FinalMesh;
			}
			void AlignMesh();
			//Eigen::Matrix4f fineAlignment(const cloudNormPtr& ptA, const cloudNormPtr& ptB, double samplRad, Eigen::Matrix4f init_tf = Eigen::Matrix4f::Identity());

		protected:
			mat4f m_AlignTransformation;
			std::vector<mat4f>m_TransformationArray;
			void computeFeature();
			algo m_algo;
			// viewPair order (src,tgt)
			std::vector<std::pair<mesh, mesh>>m_viewPairs;
			std::vector<loadStatus>m_ldstats;
			double m_samplRad, m_featRad, m_sampMul, m_featMul;
			int m_nThreads, m_nTotalView;
			mesh m_FinalMesh;
		};

	
	}
	
	
}
