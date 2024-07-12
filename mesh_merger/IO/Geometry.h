#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib> // for std::malloc and std::free

#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>


#include <openvdb/openvdb.h>
#include <openvdb/points/PointCount.h>

namespace charity
{
	namespace geometry
	{
		using vtxs = std::vector<openvdb::Vec3f>;
		using tris = std::vector<openvdb::Vec3I> ;
		using quads = std::vector<openvdb::Vec4I>;

		bool isLittleEndian();
		void writePLY(std::string fileName, const vtxs& vSet, const tris& tSet, const quads& qSet);
	
		
	}
}
