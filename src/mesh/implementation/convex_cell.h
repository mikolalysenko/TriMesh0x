
#include <vector>
#include <list>
#include <algorithm>


namespace Mesh {
namespace impl {

struct ConvexCell2D {
	typedef Eigen::Vector2f			vertex;
	typedef std::vector<vertex>		vertex_seq;
	typedef Eigen::Vector3f			halfspace;
	typedef std::list<int>			halfspace_seq;

	std::vector<halfspace>			halfspaces;
	std::vector<halfspace_seq>		contained_halfspaces;
	halfspace_seq					active;
	
	void flip_top() {
		halfspaces.back() *= -1;
		active.swap(contained_halfspaces.back());
	}

	void pop_halfspace() {
		halfspace_seq& contained = contained_halfspaces.back();
		halfspaces.pop_back();
		
		if(active.size() == 0) {
			active.swap(contained);
		} else if(contained.size() > 0) {
			auto idx = halfspaces.size();
			
			//Rotate index to front in each list
			auto iter = active.find(idx);
			active.splice(active.begin(), active, iter, active.end());
			active.pop_front()
			
			iter = contained.find(idx);
			contained.splice(contained.begin(), contained, iter, contained.end());
			contained.pop_front();
			
			auto cbegin = contained.begin();
			if(*cbegin == active.front()) {
				++cbegin;
			}
			
			auto cend = contained.end();
			if(*cend == active.back()) {
				--cend;
			}
			
			//Merge the lists back together
			active.splice(active.begin(), contained, cbegin, cend);
		}
		
		contained_halfspaces.pop_back();
	}

	void push_halfspace(halfspace const& h) {
		auto idx = halfspaces.size();
		halfspaces.push_back(idx);
		
		//Find crossing points in active halfspaces
		halfspace_seq interior, exterior;
	
	}
	
	bool check_empty() const { return active_halfspaces.size() > 0; }
	
	vertex_seq vertices() const {
		vertex_seq result;
		auto p = active.back();
		for(auto iter=active.begin(); iter!=active.end(); ++iter) {
			auto n = *iter;
			result.push_back(intersect_lines(halfspaces[p], halfspaces[n]));
			p = n;
		}
		return result;
	}
};

}}
