#pragma once

#include<iostream>
#include<vector>
#include<algorithm>
#include<Eigen/Core>

#include"../IO/defines.h"


namespace charity
{
	namespace Registration
	{
		class Registration
		{
		public:
			void AlignMesh();
			void computeFeature();
			mat4f getTransformation();


		protected:
			mat4f m_AlignTransformation;
		};
	}
}
