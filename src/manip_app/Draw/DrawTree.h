
#ifndef _CLASS_DRAWTREE
#define _CLASS_DRAWTREE

#include "MatrixDefs.h"


namespace manip_core
{
	struct TreeI;
	struct JointI;
}

class DrawTree {

public:
	 DrawTree(const manip_core::TreeI* /*tree*/, const float width = 0.02f, const float effectorWidth = 0.05f, const int id = -1);
	 DrawTree(const manip_core::TreeI* /*tree*/, const double* /*attach*/, const float width = 0.02f, const float effectorWidth = 0.05f, const int id = -1);
	~DrawTree();

public:
	void Draw(const matrices::Matrix4& /*currentTransform*/, bool transparency = false, bool wire = false, double color = -1)const;
	void DrawNoTexture(const matrices::Matrix4& /*currentTransform*/)const;

private:
	void DrawJoint(matrices::Matrix4& /*currentTransform*/, const manip_core::JointI* /*joint*/, bool wire = false, double color = -1)const;


private:
	const manip_core::TreeI* tree_;
	matrices::Vector3 attach_;
	matrices::Vector3 rootPos_;
	const matrices::Vector3 unitz_;
	const bool hasAttach_;
	const float width_;
	const float effectorWidth_;
	const int id_;
}; // class DrawTree

#endif //_CLASS_DRAWJOINT