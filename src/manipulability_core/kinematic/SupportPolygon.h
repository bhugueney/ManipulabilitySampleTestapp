
#ifndef _CLASS_SUPPORT_POLYGON
#define _CLASS_SUPPORT_POLYGON

#include "MatrixDefs.h"
#include "kinematic/Tree.h"

#include <memory>
#include <vector>

struct SupportPolygonPImpl;

class Robot;
class Tree;
class SupportPolygonVisitor_ABC;

class SupportPolygon
{

public:
	 explicit SupportPolygon(const Robot& /*robot*/); //basis switch
	~SupportPolygon();

//helper
public:
	void Accept(SupportPolygonVisitor_ABC* /*visitor*/) const;
	matrices::Vector3 Centroid() const;

//request
public:
	bool Contains(const matrices::Vector3& /*aPoint*/) const;
	bool Contains(const matrices::Vector3& /*aPoint*/, matrices::Vector3& /*correction*/) const; // returns the min translation vector
	bool WouldContain(const matrices::Vector3& /*aPoint*/, const Robot& /*robot*/, const Tree& /*tree*/); // tree can contribute to sustain com
	bool WouldContain(const matrices::Vector3& /*aPoint*/, const matrices::Vector3& /*polygonPoint*/); // tree can contribute to sustain com
	matrices::Vector3 ReplaceCom(const matrices::Vector3& com) const;
	Tree::TREE_ID ComputeTreeToLift(const matrices::Vector3& /*com*/, const matrices::Vector3& /*direction*/);
	Tree::TREE_ID ComputeTreeToLift(const matrices::Vector3& /*com*/, const matrices::Vector3& /*direction*/, const std::vector<Tree::TREE_ID>& /*candidates*/, Tree::TREE_ID /*LastLifted*/);


private:
	std::auto_ptr<SupportPolygonPImpl> pImpl_;
};

#endif //_CLASS_SUPPORT_POLYGON