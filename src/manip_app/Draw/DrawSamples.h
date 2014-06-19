
#ifndef _CLASS_DRAWSAMPLES
#define _CLASS_DRAWSAMPLES

#include "MatrixDefs.h"
#include "DrawTree.h"
#include "API/SampleVisitorI.h"
#include <vector>

namespace manip_core
{
	struct TreeI;
	struct JointI;
	struct RobotI;
}

class DrawSamples : public manip_core::SampleVisitorI {

	typedef std::pair<NUMBER,DrawTree> P_Tree;
	typedef std::vector<P_Tree> T_DrawTree;
	typedef std::vector<manip_core::TreeI*> T_Tree;

public:
	 DrawSamples(const manip_core::RobotI* /*robot*/, const manip_core::TreeI* /*tree*/, int /*id*/, bool collide = false);
	~DrawSamples();

public:
	virtual void Visit(const manip_core::RobotI* /*robot*/, manip_core::TreeI* /*tree*/);

public:
	void Draw(const matrices::Matrix4& /*currentTransform*/, bool transparency = true)const;


private:
	T_DrawTree drawTrees_;
	T_Tree trees_;
	NUMBER maxManip_;
	const int id_;
}; // class DrawTree

#endif //_CLASS_DRAWSAMPLES