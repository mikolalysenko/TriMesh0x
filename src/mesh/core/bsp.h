#ifndef MESH_BSP_H
#define MESH_BSP_H

#include <stdint.h>

#include <Eigen/Core>

#include <mesh/implementation/util.h>
#include <mesh/implementation/convex_cell.h>

namespace Mesh {

struct BSP {

	bool includes(Eigen::Vector3f const& p) const {
		Eigen::Vector4f hp(p[0], p[1], p[2], 1.0);
		int inside = 0;
		for(int i=0; i<nodes.size(); )
			inside = hp.dot(planes[nodes[i].plane_id]) < 0
			i = nodes[i].child[inside];
		}
		return inside;
	}
	
	void insert() {
		int plane_id = planes.size();
		Plane pl = poly.plane();
		planes.push_back(pl);
		
		if(nodes.size() == 0) {
			nodes.push_back( BSPNode(plane_id, -1, -1) );
			return;
		}
		
		//Compute u/v coordinates for polygon
		Eigen::Vector3f du, dv, p0;
		
		
		//Stack
		std::vector<int> node;
		
		while(stack.size() > 0) {
			auto frame = stack.back();
			stack.pop_back();
			
			//Flip the last plane on the stack
			hspaces.back() = -hspaces.back();
		
			//Read the plane
			auto pl = planes[nodes[frame.node].plane_id];
			
			//Project into u/v space
			Eigen::Vector3f pl_uv = project_uv(pl, du, dv, p0);
			
			//Append to plane stack
			hspaces.push_back(pl_uv);
			
			//Intersect with current plane
			float t = intersect_planes(pl_uv, nodes[cur_plane]);
			
			
			
		}
	}
	
private:


	struct BSPNode {
		uint16_t plane_id, child[2];
	};
	
	std::vector<BSPNode>			nodes;
	std::vector<Plane>				planes;
};

template<typename Mesh_t>
BSP make_bsp(Mesh_t const& mesh, std::vector<int> const& tris) {
	BSP result;
	
	if(tris.size() > 0) {
	
	} else {
	
	}
	
	return result;
}

};

#endif

