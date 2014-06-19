
#ifndef _CLASS_WORLDPARSEROBJ
#define _CLASS_WORLDPARSEROBJ

#include "MatrixDefs.h"
#include "ManipManager.h"

#include <string>
#include <vector>

struct WorldParserObj
{
public:
	 WorldParserObj();
	~WorldParserObj();

	void CreateWorld(const std::string& /*filename*/, const bool isGround=false);

private:
	void CreateObstacle (const std::vector<std::string>& /*lines*/, const bool /*isGround*/);
	
private:
	typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices::Vector3>> T_Vector3;

private:
	manip_core::ManipManager& manager_;
	T_Vector3 points_;
	T_Vector3 normals_;
};

#endif //_CLASS_WORLDPARSEROBJ