
#ifndef _CLASS_DRAWTRAJECTORY
#define _CLASS_DRAWTRAJECTORY

#include "MatrixDefs.h"

#include <vector>

class DrawTrajectory{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 DrawTrajectory();
	~DrawTrajectory();

	void AddPoint(const matrices::Vector3& point);
	void AddPoint(const float* point);

	 void Draw() const;
	 void Clear();

private:
	std::vector<matrices::Vector3> points_;
}; // class DrawWorld

#endif //_CLASS_DRAWTRAJECTORY