#include"Geometry.h"



bool charity::geometry::isLittleEndian()
{
	unsigned int tmp = 1;
	return (*(char*)&tmp == 1);
}
void charity::geometry::writePLY(std::string fileName, const vtxs& vSet, const tris& tSet, const quads& qSet)
{
	std::ofstream os(fileName, std::ios_base::binary);
	if (!os.is_open()) throw std::invalid_argument("Error writing to ply file \"" + fileName + "\"");
	os << "ply\n";
	if (isLittleEndian()) {
		os << "format binary_little_endian 1.0\n";
	}
	else {
		os << "format binary_big_endian 1.0\n";
	}
	os << "comment created by vdb_tool" << std::endl;
	os << "element vertex " << vSet.size() << std::endl;
	os << "property float x\n";
	os << "property float y\n";
	os << "property float z\n";
	os << "element face " << (tSet.size() + 2 * qSet.size()) << std::endl;
	os << "property list uchar int vertex_index\n";
	os << "end_header\n";

	// write vertex
	static_assert(sizeof(openvdb::Vec3s) == 3 * sizeof(float), "Unexpected sizeof(Vec3s)");
	os.write((const char*)vSet.data(), vSet.size() * 3 * sizeof(float));

	// write tri-faces
	if (tSet.size() > 0) {
		const size_t size = sizeof(char) + 3 * sizeof(uint32_t);
		char* buffer = static_cast<char*>(std::malloc(tSet.size() * size)), * p = buffer;// uninitialized
		if (buffer == nullptr) throw std::invalid_argument("Geometry::writePLY: failed to allocate buffer");
		static_assert(sizeof(openvdb::Vec3I) == 3 * sizeof(uint32_t), "Unexpected sizeof(Vec3I)");
		for (const openvdb::Vec3I* t = tSet.data(), *e = t + tSet.size(); t != e; ++t) {
			*p = 3;
			openvdb::Vec3I t_rev = t->reversed();
			std::memcpy(p + 1, (void*)&t_rev, 3 * sizeof(uint32_t));
			p += size;
		}
		os.write(buffer, tSet.size() * size);
		std::free(buffer);
	}

	// write quad-faces
	if (qSet.size() > 0) {
		const size_t size = sizeof(char) + 3 * sizeof(uint32_t);
		char* buffer = static_cast<char*>(std::malloc( 2 * qSet.size() * size)), * p = buffer;// uninitialized
		if (buffer == nullptr) throw std::invalid_argument("Geometry::writePLY: failed to allocate buffer");
		static_assert(sizeof(openvdb::Vec4I) == 4 * sizeof(uint32_t), "Unexpected sizeof(Vec4I)");
		for (const openvdb::Vec4I* q = qSet.data(), *e = q + qSet.size(); q != e; ++q) {
			*p = 3;
			//*qq = 3;
			//std::cout << q->x() << q->y() << q->z() << q->w() << std::endl;
			//openvdb::Vec4I q_rev(q->w(), q->z(), q->y(), q->x());
			openvdb::Vec3I q_rev_1(q->w(), q->z(), q->y());
			openvdb::Vec3I q_rev_2(q->x(),  q->w(), q->y());
			//std::cout << q_rev << std::endl;
			//std::memcpy(p + 1, (void*)&q_rev, 4 * sizeof(uint32_t));
			std::memcpy(p + 1, (void*)&q_rev_1, 3 * sizeof(uint32_t));
			p += size;
			*p = 3;
			std::memcpy(p + 1, (void*)&q_rev_2, 3 * sizeof(uint32_t));
			//qq += size;
			p += size;
			
		}
		os.write(buffer, 2  * qSet.size() * size);
		std::free(buffer);
	}
}