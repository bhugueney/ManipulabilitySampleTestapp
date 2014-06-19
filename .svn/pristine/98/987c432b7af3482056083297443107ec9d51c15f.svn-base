
#ifndef _CLASS_FILTER_DISTANCE
#define _CLASS_FILTER_DISTANCE

#include <memory>

#include "Filter_ABC.h"
#include "MatrixDefs.h"

class Sample;
class Tree;
class Obstacle;


// checks that end-effector is "around" obstacle
class FilterDistance : public Filter_ABC {

public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 FilterDistance(NUMBER /*treshold*/, const Tree& /*tree*/, const matrices::Vector3& /*target*/); // in robot coordinates
	~FilterDistance();

protected:
	virtual bool ApplyFilter(const Sample& /*sample*/) const;

private:
	matrices::Vector3 target_;
	matrices::Vector3 treePos_;
	NUMBER treshold_;
};


#endif //_CLASS_FILTER_DISTANCE