#ifndef CONVEX_CELL_H
#define CONVEX_CELL_H

#include <vector>
#include <algorithm>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace Mesh {
namespace impl {


/**
 * Used to implement incremental linear programming and face extraction in BSP trees.
 */
struct ConvexCell2D {

	typedef Eigen::Hyperplane<float, 2>		halfspace;
	typedef std::vector<halfspace, Eigen::aligned_allocator<halfspace> > halfspace_seq;
	typedef halfspace::VectorType			vertex;
	typedef halfspace::VectorType			normal;
	typedef std::vector<vertex, Eigen::aligned_allocator<vertex> > vertex_seq;
	typedef std::vector<int>				cycle;
	typedef std::vector<cycle>				cycle_stack;

	halfspace_seq	halfspaces;
	cycle_stack		contained_halfspaces;
	cycle			active;
	
	enum PMC_VAL {
		PMC_IN			= 1,
		PMC_OUT			= 2,
		PMC_ON			= 4,
	};
	
	//Note that the cell needs to have 3
	ConvexCell2D() : halfspaces({
		halfspace(normal( 1., 0.), 1000.),
		halfspace(normal( 0., 1.), 1000.),
		halfspace(normal(-1., 0.), 1000.),
		halfspace(normal( 0.,-1.), 1000.)}) {
		
		cycle tmp;
		for(int i=0; i<halfspaces.size(); ++i) {
			active.push_back(i);
			contained_halfspaces.push_back(tmp);
		}
	}
	ConvexCell2D(const halfspace_seq& p) : halfspaces(p) {
		cycle tmp;
		for(int i=0; i<halfspaces.size(); ++i) {
			active.push_back(i);
			contained_halfspaces.push_back(tmp);
		}
	}
	ConvexCell2D(halfspace_seq&& p) : halfspaces(p) {
		cycle tmp;
		for(int i=0; i<halfspaces.size(); ++i) {
			active.push_back(i);
			contained_halfspaces.push_back(tmp);
		}
	}
	
	//Boilerplate constructors
	ConvexCell2D(const ConvexCell2D& c) :
		halfspaces(c.halfspaces),
		contained_halfspaces(c.contained_halfspaces),
		active(c.active) {}
	ConvexCell2D(ConvexCell2D&& c) :
		halfspaces(c.halfspaces),
		contained_halfspaces(c.contained_halfspaces),
		active(c.active) {}
	ConvexCell2D& operator=(const ConvexCell2D& c) {
		halfspaces = c.halfspaces;
		contained_halfspaces = c.contained_halfspaces;
		active = c.active;
		return *this;
	}
	ConvexCell2D& operator=(ConvexCell2D&& c) {
		halfspaces = c.halfspaces;
		contained_halfspaces = c.contained_halfspaces;
		active = c.active;
		return *this;
	}
	
	//Classify a point
	PMC_VAL classify_point(int i, int j, int k) const {
		halfspace tmp = halfspaces[j];
		auto v = tmp.intersection(halfspaces[k]);
		float d = halfspaces[i].signedDistance(v);
		std::cout << "v = " << v << ", d = " << d << std::endl;
		if(d < -FP_TOLERANCE)
			return PMC_IN;
		if(d >  FP_TOLERANCE)
			return PMC_OUT;
		return PMC_ON;
	}
	
	void flip_back() {
		halfspaces.back().coeffs() *= -1;
		active.swap(contained_halfspaces.back());
	}

	void pop_back() {
		cycle& contained = contained_halfspaces.back();
		halfspaces.pop_back();
		
		if(active.empty()) {
			active.swap(contained);
		} else if(!contained.empty()) {
			auto idx = halfspaces.size();
			
			
			std::cout << "Removing plane: " << idx << std::endl;
			std::cout << "Active = ";
			for(int i=0; i<active.size(); ++i)
				std::cout << active[i] << ',';
			std::cout << std::endl << "Contained = ";
			for(int i=0; i<contained.size(); ++i)
				std::cout << contained[i] << ',';
			std::cout << std::endl;
			
			//Vector version
			int pend = active.size();
			active.resize(contained.size() + active.size() - 4);
			auto a_iter = std::find(active.begin(), active.end(), idx);
			auto a_end = active.begin() + pend;
			auto b_start = contained.begin();
			auto b_iter = std::find(contained.begin(), contained.end(), idx);
			
			assert(a_iter != active.end() && b_iter != contained.end());
			
			if(a_iter+1 != active.end()) {
				std::copy_backward(a_iter+2, a_end, active.end());
			}
			if(b_iter+1 != contained.end()) {
				a_iter = std::copy(b_iter+2, contained.end(), a_iter);
			}
			else {
				++b_start;
			}
			std::copy(b_start, b_iter, a_iter);
		}
		
		std::cout << "Resulting Active Cycle = ";
		for(int i=0; i<active.size(); ++i)
			std::cout << active[i] << ',';
		std::cout << std::endl;

		
		contained_halfspaces.pop_back();
	}

	//Pushes a halfspace onto the plane stack; partition the current active cell
	// (This is the hard part)
	void push_back(halfspace const& h) {
		//Do initial set up stuff
		auto idx = halfspaces.size();
		halfspaces.push_back(h);
		const int N = active.size();
		
		//Empty
		if(N == 0) {
			contained_halfspaces.push_back(active);
			return;
		}
		
		std::cout << "----------------------------Adding halfspace, h = " << h.coeffs() << std::endl;
		
		//Classify all vertices		
		std::vector<PMC_VAL> codes(N);
		auto iter = active.begin();
		for(int i=0; iter!=active.end(); ++iter, ++i) {
			auto next = iter;
			++next;
			if(next == active.end()) {
				next = active.begin();
			}
			std::cout << "Classifying point " << i << std::endl;
			codes[i] = classify_point(idx, *iter, *next);
		}
		
		//Compute predecessor and successor codes by dynamic programming
		std::vector<PMC_VAL> 
			pred_codes, 
			succ_codes(codes.begin(), codes.end());
		pred_codes.reserve(N);
		pred_codes.push_back(codes.back());
		pred_codes.insert(pred_codes.end(), codes.begin(), codes.end()-1);
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
		cycle interior;
		contained_halfspaces.push_back(interior);
		cycle& exterior = contained_halfspaces.back();
		interior.reserve(active.size()+1);
		exterior.reserve(active.size()+1);

		auto cur = active.begin();		
		for(int i=0, p=codes.back(); i<codes.size(); ++i, ++cur) {
		
			int a = pred_codes[i],
				b = succ_codes[i],
				n = codes[i];
			
			//Printing stuff
			halfspace tmp = halfspaces[*cur];
			halfspace nh, ph;
			if(cur == active.end()-1) {
				nh = halfspaces[active.front()];
			}
			else {
				auto t2 = cur;
				++t2;
				nh = halfspaces[*t2];
			}
			if(cur == active.begin()) {
				ph = halfspaces[active.back()];
			}
			else {
				auto t2 = cur;
				--t2;
				ph = halfspaces[*t2];
			}
			std::cout << "Classifying edge " << i << ':' << std::endl 
				<< "plane = " << tmp.coeffs() << std::endl
				<< "v0 = " << tmp.intersection(ph) << std::endl
				<< "v1 = " << tmp.intersection(nh) << std::endl
				<< "(p, n, pred, succ) = (" << p << ',' << n << ',' << a << ',' << b << ")" << std::endl;
			
			if(a == PMC_IN) {
				std::cout << "+interior" << std::endl;
				interior.push_back(*cur);
				if(n == PMC_OUT) {
					std::cout << "+h-exterior" << std::endl;
					exterior.push_back(idx);
					std::cout << "+exterior" << std::endl;
					exterior.push_back(*cur);
				}
			}
			else if(a == PMC_OUT) {
				std::cout << "+exterior" << std::endl;
				exterior.push_back(*cur);
				if(n == PMC_IN) {
					std::cout << "+h-interior" << std::endl;
					interior.push_back(idx);
					std::cout << "+interior" << std::endl;
					interior.push_back(*cur);
				}
			}
			p = n;
		}
		
		std::cout << "Interior = " << std::endl;
		if(interior.empty()) {
			std::cout << "empty" << std::endl;
		}
		else {
			auto p = interior.back();
			for(auto iter=interior.begin(); iter!=interior.end(); ++iter) {
				auto n = *iter;
				halfspace a = halfspaces[p], b = halfspaces[n];
				std::cout << a.intersection(b) << ',' << std::endl;
				p = n;
			}
		}

		active.swap(interior);
	}
	
	bool empty() const { return active.empty(); }
	
	vertex_seq vertices() const {
		vertex_seq result;
		if(active.empty()) 
			return result;
		auto p = active.back();
		for(auto iter=active.begin(); iter!=active.end(); ++iter) {
			auto n = *iter;
			halfspace a = halfspaces[p], b = halfspaces[n];
			result.push_back(a.intersection(b));
			p = n;
		}
		return result;
	}
};

}}

#endif

