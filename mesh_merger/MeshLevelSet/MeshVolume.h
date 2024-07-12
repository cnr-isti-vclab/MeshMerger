#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

#include "boost/filesystem.hpp"
#include "boost/endian.hpp"

#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"

#include "openvdb/openvdb.h"
#include "openvdb/io/Stream.h"
#include "openvdb/tools/MeshToVolume.h"
#include "openvdb/tools/Dense.h"
#include "openvdb/tools/TopologyToLevelSet.h"
#include "openvdb/io/Stream.h"
#include "openvdb/tools/VolumeToMesh.h"


#include"../IO/defines.h"
#include"../IO/Geometry.h"

#include "CLI11.hpp"

typedef openvdb::FloatGrid::Ptr openVDBGrid;
typedef openvdb::math::Transform::Ptr openVDBTransform;

namespace charity
{   
    namespace vdb
    {
        typedef struct 
        {
            std::vector<openvdb::Vec3s> points;
            std::vector<openvdb::Vec3I> faces;
            openvdb::BBoxd bbox;
            std::vector<openvdb::Vec4I> quads;
           
        }vMesh;

        typedef struct 
        {
            std::vector<openvdb::Vec3f> points;
            std::vector<openvdb::Vec3I> triangles;
            std::vector<openvdb::Vec4I> quads;
            float isoValue;
            float adapt;
        }VolumetricMesh;

        typedef struct 
        {
            bool flag_silent = 0, flag_fill = 0, flag_full = 0, flag_world = 1,
                flag_dense = 0, flag_unsigned = 0, flag_index = 0, flag_cube = 1, flag_cell = 1;
        }MeshVolumeFlags;
        

        class cMeshVolume
        {
        public:
            cMeshVolume();
            cMeshVolume(const mesh& alignedMesh, float isValue, float adaptive, int dim = 128, int bw = 3, std::string outFile = "");
            void dataConversion(const mesh& alignedMesh);
            void computeParams();
            void buildGrid();
            void convertUsdfToMesh(bool flagRelax = true,bool write =false);
        private:
            openVDBGrid vGrid;
            vMesh volMesh;
            vMesh outVolMesh;
            int gridDimension;
            int narrowbandWidth;
            openvdb::Vec3d center;
            openvdb::Vec3d extent;
            float voxelSize;
            openvdb::Vec3d volumeSize;
            MeshVolumeFlags v_Flags;
            openVDBTransform transform;
            openVDBGrid grid;
            float exbw;
            float inbw;
            float isoValue;
            float adapt;
            std::string outFileName;
        };
    }
}
