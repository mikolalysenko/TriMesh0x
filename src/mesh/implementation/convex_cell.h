#ifndef CONVEX_CELL_H
#define CONVEX_CELL_H

#include <vector>
#include <list>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace Mesh {
namespace impl {


//Used to implement incremental linear programming and face extraction in BSP trees
struct ConvexCell2D {

	typedef Eigen::Hyperlane<float, 2>		halfspace;
	typedef std::vector<halfspace, Eigen::aligned_allocator<halfspace> > halfspace_seq;
	typedef typename halfspace::VectorType	vertex;
	typedef std::vector<vertex, Eigen::aligned_allocator<vertex> > vertex_seq;
	typedef std::list<int>					cycle;
	typedef std::vector<cycle>				cycle_stack;

	halfspace_seq	halfspaces;
	cycle_stack		coincident_halfspaces;
	cycle_stack		contained_halfspaces;
	cycle			active;
	
	enum PMC_VAL {
		PMC_IN			= 1,
		PMC_OUT			= 2,
		PMC_ON			= 4,
		PMC_COPLANAR	= 8,
	};
	
	enum PLANE_VAL {
		PLANE_GENERAL			= 0,
		PLANE_INTERIOR_COPLANAR	= 1,
		PLANE_EXTERIOR_COPLANAR	= 2,
		PLANE_COINCIDENT		= 3
	}
	
	ConvexCell2D() : halfspaces({
		halfspace( 1, 0, 1000),
		halfspace( 0, 1, 1000),
		halfspace(-1, 0, 1000),
		halfspace( 0,-1, 1000)}) {
		for(int i=0; i<halfspaces.size(); ++i) {
			active.push_back(i);
		}
	}
	ConvexCell2D(const halfspace_seq& p) : halfspaces(p) {
		for(int i=0; i<halfspaces.size(); ++i) {
			active.push_back(i);
		}
	}
	ConvexCell2D(halfspace_seq&& p) : halfspaces(p) {
		for(int i=0; i<halfspaces.size(); ++i) {
			active.push_back(i);
		}
	}
	ConvexCell2D(const ConvexCell2D& c) :
		halfspaces(c.halfspaces),
		coincident_halfpsaces(c.coincident_halfspaces),
		contained_halfspaces(c.contained_halfspaces),
		active(c.active) {}
	ConvexCell2D(ConvexCell&& c) :
		halfspaces(c.halfspaces),
		coincident_halfspaces(c.coincident_halfspaces),
		contained_halfspaces(c.contained_halfspaces),
		active(c.active) {}
	ConvexCell2D& operator=(const ConvexCell2D& c) {
		halfspaces = c.halfspaces;
		coincident_halfspaces = c.coincident_halfspaces;
		contained_halfspaces = c.contained_halfspaces;
		active = c.active;
		return *this;
	}
	ConvexCell2D& operator=(ConvexCell2D&& c) {
		halfspaces = c.halfspaces;
		coincident_halfspaces = c.coincident_halfspaces;
		contained_halfspaces = c.contained_halfspaces;
		active = c.active;
		return *this;
	}
	
	
	//Classify a point
	PMC_VAL classify_point(int i, int j, int k) const {
		float d = halfspaces[i].signedDistance(halfspaces[j].intersection(halfspaces[k]));
		if(d < -FP_TOLERANCE)
			return PMC_IN;
		if(d >  FP_TOLERANCE) {
			return PMC_OUT;
		return PMC_ON;
	}
	
	PLANE_VAL classify_plane(int i, int j) const {
		return PLANE_GENERAL;
	}
	
	void flip_top() {
		halfspaces.back() *= -1;
		active.swap(contained_halfspaces.back());
	}

	void pop_halfspace() {
		halfspace_seq& contained = contained_halfspaces.back();
		halfspaces.pop_back();
		
		if(active.empty()) {
			active.swap(contained);
		} else if(!contained.empty()) {
			auto idx = halfspaces.size();
			
			//Rotate index to front in each list
			auto iter = active.find(idx);
			active.splice(active.begin(), active, iter, active.end());
			active.pop_front();
			
			iter = contained.find(idx);
			contained.splice(contained.begin(), contained, iter, contained.end());
			contained.pop_front();
			
			auto cbegin = contained.begin();
			auto ptr = active.begin();
			while(true) {
				if(*cbegin == *ptr) {
					++cbegin;
				}
				else {
					++ptr;
					if(*cbegin == *ptr) {
						++cbegin;
					}
					else {
						break;
					}
				}
			}
			
			auto cend = contained.end();
			ptr = active.end();
			while(true) {
				if(*cend == *ptr) {
					--cend;
				}
				else {
					--ptr;
					if(*cend == *ptr) {
						--cend;
					}
					else {
						break;
					}
				}
			}
			
			//Merge the lists back together
			active.splice(active.begin(), contained, cbegin, cend);
		}
		
		contained_halfspaces.pop_back();
	}

	//Pushes a halfspace onto the plane stack; partition the current active cell
	// (This is the hard part)
	void push_halfspace(halfspace const& h) {
		auto idx = halfspaces.size();
		halfspaces.push_back(idx);
		
		//Get size		
		int N = active.size();
		
		//Classify all vertices		
		std::vector<PMC_VAL> codes(N);
		auto iter = active.begin();
		bool all_on = true;
		for(int i=0; iter!=active.end(); ++iter) {
			auto next = iter;
			++next;		
			codes[i] = classify_point(idx, *iter, *next);
			all_on &= (codes[i] == PMC_ON);
		}
		
		//Compute predecessor and successor codes by dynamic programming
		std::vector<PMC_VAL> 
			pred_codes(codes.begin()+1, codes.end()), 
			succ_codes(codes.begin(), codes.end());
		pred_codes.push_back(codes.front());
		if(pred_codes.front() == PMC_ON) {
			for(int j=pred_codes.size()-1; j>=0; --j) {
				if(pred_codes[j] != PMC_ON) {
					pred_codes[0] = pred_codes[j];
					break;
				}
			}
		}
		for(int i=1; i<pred_codes.size(); ++i) {
			if(pred_codes[i] == PMC_ON) {
				pred_codes[i] = pred_codes[i-1];
			}
		}
		if(succ_codes.back() == PMC_ON) {
			for(int j=0; j<succ_codes.size(); ++j) {
				if(succ_codes[j] != PMC_ON) {
					succ_codes[0] = succ_codes[j];
					break;
				}
			}
		}
		for(int i=succ_codes.size()-2; i>=0; --i) {
			if(succ_codes[i] == PMC_ON) {
				succ_codes[i] = succ_codes[i+1];
			}
		}
		
		//Partition the edge set into interior and exterior by h
		halfspace_seq interior;
		contained_halfspaces.push_back(interior);
		coincident_halfspaces.push_back(interior);
		halfspace_seq& exterior = contained_halfspaces.back();
		halfspace_seq& incident = coincident_halfspaces.back();
		
		auto cur = active.begin();		
		for(int i=0, p=codes.back(); i<codes.size(); ++i, ++cur) {
		
			int a = pred_codes[i],
				b = succ_codes[i],
				n = codes[i];
			
			if((p|n) == PMC_ON) {
				incident.push_back(*cur);
			}
			else if(a == PMC_IN) {
				interior.push_front(*cur);
				if(b == PMC_OUT) {
					interior.push_front(idx);
					if(n != PMC_ON) {
						exterior.push_front(idx);
					}
					exterior.push_front(*cur);
				}
			}
			else if(a == PMC_OUT) {
				exterior.push_front(*cur);
				if(b == PMC_IN) {
					exterior.push_front(idx);
					if(n != PMC_ON) {
						interior.push_front(idx);
					}
					interior.push_front(*cur);
				}
			}
			
			p = n;
		}
		
		active.swap(interior);
	}
	
	bool empty() const { return active_halfspaces.empty(); }
	
	vertex_seq vertices() const {
		vertex_seq result;
		auto p = active.back();
		for(auto iter=active.begin(); iter!=active.end(); ++iter) {
			auto n = *iter;
			result.push_back(halfspaces[p].intersection(halfspaces[n]));
			p = n;
		}
		return result;
	}
};

}}

#endif

