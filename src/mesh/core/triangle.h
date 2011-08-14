#ifndef MESH_TRIANGLE_H
#define MESH_TRIANGLE_H

#include <vector>

#include "mesh/implementation/util.h"

namespace Mesh {


/**
 * A triangle.
 * 
 * This is a simple wrapper struct which stores the 3 indexes for the triangle's
 * vertices, and implements a few miscellaneous helper methods.  This struct is
 * primarily used internally within the TriMesh.
 */
template<typename UserData_t>
struct Triangle {


	typedef UserData_t UserData;

	/// The vertex indices for the triangle
	int v[3];

	//Boiler plate constructors
	Triangle() {}
	Triangle(const Triangle& o) : data_(o.data_) {
		v[0] = o.v[0];
		v[1] = o.v[1];
		v[2] = o.v[2];
	}
	Triangle(Triangle&& o) : data_(o.data_) {
		v[0] = o.v[0];
		v[1] = o.v[1];
		v[2] = o.v[2];
	}
	Triangle& operator=(const Triangle& o) {
		v[0] = o.v[0];
		v[1] = o.v[1];
		v[2] = o.v[2];
		data_ = o.data_;
		return *this;		
	}

	//User level constructors
	Triangle(int v0, int v1, int v2, UserData const& d) : data_(d) {
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
	}
	Triangle(int v0, int v1, int v2, UserData&& d = UserData() ) : data_(d) {
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
	}
	Triangle(int v0, int v1, int v2) {
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
	}
	Triangle(const int* ptr, UserData const& d) : data_(d) {
		v[0] = ptr[0];
		v[1] = ptr[1];
		v[2] = ptr[2];
	}
	Triangle(const int* ptr, UserData&& d) : data_(d) {
		v[0] = ptr[0];
		v[1] = ptr[1];
		v[2] = ptr[2];
	}
	Triangle(const int* ptr) {
		v[0] = ptr[0];
		v[1] = ptr[1];
		v[2] = ptr[2];
	}
	Triangle(std::vector<int> const& vec, UserData const& d) : data_(d) {
		v[0] = vec[0];
		v[1] = vec[1];
		v[2] = vec[2];
	}
	Triangle(std::vector<int> const& vec, UserData&& d) : data_(d) {
		v[0] = vec[0];
		v[1] = vec[1];
		v[2] = vec[2];
	}
	Triangle(std::vector<int> const& vec) {
		v[0] = vec[0];
		v[1] = vec[1];
		v[2] = vec[2];
	}

	///Returns the index of the vertex v_ in the index buffer
	int index_of(int v_) const {
		for(int i=0; i<3; ++i) {
			if(v[i] == v_) {
				return i;
			}
		}
		return -1;
	}
	
	///Returns a vector for the vertex list of the triangle
	std::vector<int> vlist() const {
		return std::vector<int>(v, v+3); 
	}
	
	//Data accessor
	UserData const& data() const { return data_; }
	UserData& data() { return data_; }
	
private:
	///User specific data
	UserData data_;
};

/**
 * Specialization:  When no additional data is required, we avoid storing the additional user data struct
 */
template<> struct Triangle<Mesh::EmptyStruct> {

	typedef EmptyStruct UserData;

	/// The vertex indices for the triangle
	int v[3];

	//Boiler plate constructors
	Triangle() {}
	Triangle(const Triangle& o) {
		v[0] = o.v[0];
		v[1] = o.v[1];
		v[2] = o.v[2];
	}
	Triangle(Triangle&& o) {
		v[0] = o.v[0];
		v[1] = o.v[1];
		v[2] = o.v[2];
	}
	Triangle& operator=(const Triangle& o) {
		v[0] = o.v[0];
		v[1] = o.v[1];
		v[2] = o.v[2];
		return *this;		
	}

	//User level constructors
	Triangle(int v0, int v1, int v2, EmptyStruct const& d) {
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
	}
	Triangle(const int* ptr, EmptyStruct const& d) {
		v[0] = ptr[0];
		v[1] = ptr[1];
		v[2] = ptr[2];
	}
	Triangle(std::vector<int> const& vec, EmptyStruct const& d) {
		v[0] = vec[0];
		v[1] = vec[1];
		v[2] = vec[2];
	}
	Triangle(int v0, int v1, int v2) {
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
	}
	Triangle(const int* ptr) {
		v[0] = ptr[0];
		v[1] = ptr[1];
		v[2] = ptr[2];
	}
	Triangle(std::vector<int> const& vec) {
		v[0] = vec[0];
		v[1] = vec[1];
		v[2] = vec[2];
	}

	///Returns the index of the vertex v_ in the index buffer
	int index_of(int v_) const {
		for(int i=0; i<3; ++i) {
			if(v[i] == v_) {
				return i;
			}
		}
		return -1;
	}
	
	///Returns a vector for the vertex list of the triangle
	std::vector<int> vlist() const {
		return std::vector<int>(v, v+3); 
	}
	
	//Data accessors
	EmptyStruct const& data() const { 
		static EmptyStruct garbage;	
		return garbage;
	}
	EmptyStruct data() {
		static EmptyStruct garbage;
		return garbage;
	}
};

template<typename UserData=EmptyStruct> class Triangle;

};

#endif

