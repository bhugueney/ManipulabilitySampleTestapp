
#include "DrawTree.h"
#include "MatrixDefs.h"

#include "API/RobotI.h"
#include "API/TreeI.h"
#include "API/JointI.h"
#include "Pi.h"
#include "Simulation.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

using namespace matrices;
using namespace manip_core;

namespace matrices
{
	const Vector3 unitz(0,0,1);
}

DrawTree::DrawTree(const TreeI* tree, const float width, const float effectorWidth, const int id)
	: tree_(tree)
	, hasAttach_(false)
	, width_(width)
    , effectorWidth_(effectorWidth)
	, id_(id)
{
	// NOTHING
}

DrawTree::DrawTree(const TreeI* tree, const double* attach, const float width, const float effectorWidth, const int id)
	: tree_(tree)
	, hasAttach_(true)
	, width_(width)
    , effectorWidth_(effectorWidth)
	, id_(id)
{
	const JointI* joint = this->tree_->GetRootJointI();
	double vect[3];
	joint->Offset(vect);
	matrices::arrayToVect3(vect, rootPos_);
	matrices::arrayToVect3(attach, attach_);
}


DrawTree::~DrawTree()
{
	// NOTHING
}

void DrawTree::Draw(const matrices::Matrix4& currentTransform, bool transparency, bool wire, double color) const
{
	Vector3 endPosition; // drawing contact point
	matrices::Matrix4 m = currentTransform;
	dsSetColor(1.0,0.0,0.0);
	const JointI* joint = this->tree_->GetRootJointI();
	//if(tree_->IsAnchored() &! transparency)
	if(!transparency)
	{
		dsSetTexture(1);
		dsSetColor(1.0,1.0,1.0);
		//dsSetColor(1.0,1.0,1.0);
	}
	else if (! wire)
	{
		dsSetTexture(0);
		dsSetColorAlpha(0.84f,0.7f,0.62f, 0.4f);
		//dsSetColorAlpha(1.0,1.0,1.0,0.6);
	}
	else 
	{
		dsSetTexture(0);
		//dsSetColor(0.3, 0.3, 0.6);
		dsSetColorAlpha(1.0,1.0,1.0,0.6);
		dsSetColorAlpha(0.54f,0.4f,0.32f, 0.8f);
	}
	if(hasAttach_)
	{
		Vector3 vFrom(matrices::matrix4TimesVect3(currentTransform, attach_));
		Vector3 vTo(matrices::matrix4TimesVect3(currentTransform, rootPos_));
		float from[3];
		float to[3];
		float R[12];
		matrices::vect3ToArray(from, vFrom);
		matrices::vect3ToArray(to, vTo);
		dsDrawLine(from, to);

		Vector3 trans(vTo - vFrom);
		
		matrices::vect3ToArray(from, vFrom + trans / 2.);
		trans.normalize();
		Matrix3 transformCube;
		GetRotationMatrix(unitz, trans, transformCube);
		matrices::Matrix3 mr2 = matrices::Matrix3::Zero();
		mr2(0,0) = 0.2;
		mr2(1,1) = 0.2;
		mr2(2,2) = 1;
		matrix3ToArray(R, transformCube * mr2);
		//dsDrawCapsule(from, R, (float)((vTo - vFrom).norm() - 0.01f), width_);
		if(!wire || Simulation::GetInstance()->simpParams_.drawArms_)
		{
			//dsDrawCylinder(from, R, (float)((vTo - vFrom).norm() - 0.01f) , 0.01);
			dsDrawSphere(from, R, (float)((vTo - vFrom).norm() - 0.01f) / 2);
		}
	}
	while(joint)
	{
		DrawJoint(m, joint, wire, color);
		joint = (joint->IsEffector() ? 0 : joint->GetSon());
	}
	/*if tree has an obstacle, draw a contact member*/
		//dsSetColor(1.0,1.0,1.0);
}

void DrawTree::DrawNoTexture(const matrices::Matrix4& currentTransform) const
{
	matrices::Matrix4 m = currentTransform;
	dsSetColor(1.0,0.0,0.0);
	const JointI* joint = this->tree_->GetRootJointI();
	if(tree_->IsAnchored())
	{
		dsSetColorAlpha(1.0,1.0,1.0, 0.3);
		//dsSetColorAlpha(1.0,1.0,1.0, 0.3);
		//dsSetColor(1.0,1.0,1.0);
	}
	else
	{
		dsSetColorAlpha(1.0,1.0,1.0, 0.3);
	}
	if(hasAttach_)
	{
		Vector3 vFrom(matrices::matrix4TimesVect3(currentTransform, attach_));
		Vector3 vTo(matrices::matrix4TimesVect3(currentTransform, rootPos_));
		float from[3];
		float to[3];
		float R[12];
		matrices::vect3ToArray(from, vFrom);
		matrices::vect3ToArray(to, vTo);
		dsDrawLine(from, to);

		Vector3 trans(vTo - vFrom);
		
		matrices::vect3ToArray(from, vFrom + trans / 2.);
		trans.normalize();
		Matrix3 transformCube;
		GetRotationMatrix(unitz, trans, transformCube);
		matrix3ToArray(R, transformCube);
		//dsDrawCylinder(from, R, (float)((vTo - vFrom).norm() - 0.01f), width_);
		dsDrawCylinder(from, R, (float)((vTo - vFrom).norm()), width_);
	}
	while(joint)
	{
		DrawJoint(m, joint);
		joint = (joint->IsEffector() ? 0 : joint->GetSon());
	}
		dsSetColor(1.0,1.0,1.0);
}


//void DrawTree::Draw(const matrices::Matrix4& currentTransform) const
//{
//	matrices::Matrix4 m = currentTransform;
//		
//	if(tree_->IsLocked())
//	{
//		dsSetColor(0.0,1.0,0.0);
//		// draw target
//		float ps[3];
//		float Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
//		vect3ToArray(ps, tree_->GetTarget());
//		dsDrawSphere (ps, Identity, 0.05f);
//	}
//	else
//	{
//		dsSetColor(1.0,0.0,0.0);
//	}
//	Joint* joint = tree_->GetRoot();
//	while (joint)
//	{
//		DrawJoint(m, joint);
//		joint = (joint->IsEffector() ? 0 : tree_->GetSuccessor(joint));
//	}
//}

using namespace enums;

void DrawTree::DrawJoint(matrices::Matrix4& currentTransform, const JointI* joint, bool wire, double color) const
{
	//draw line between positions
	Vector3 vFrom(currentTransform.block<3,1>(0,3));
	float from[3];
	vect3ToArray(from, vFrom);
	double vect[3];
	joint->Offset(vect);
	Vector3 offset;
	matrices::arrayToVect3(vect,offset);
	Matrix4 jointTransform = Translate(offset); // translation of current joint
	switch(joint->GetRotation())
	{
		case rotation::X:
			jointTransform = jointTransform + Rotx4(joint->GetAngle());
			break;
		case rotation::Y:
			jointTransform = jointTransform + Roty4(joint->GetAngle());
			break;
		case rotation::Z:
			jointTransform = jointTransform + Rotz4(joint->GetAngle());
			break;
	}
	currentTransform = currentTransform * jointTransform;
	float R[12];
	matrixToArray(R, currentTransform);
	float ps[3];

	vect4ToArray(ps, currentTransform.col(3));
	if(joint->GetSon())
	{
		if(!joint->IsLocked() )
		{
			//dsSetTexture(1);
			//dsSetColorAlpha(1,1,1,1.0);
			if(!wire || Simulation::GetInstance()->simpParams_.drawArms_)
			//dsDrawSphere (ps, R, 0.05f);
			dsDrawSphere (ps, R, 0.02f);
			//dsSetTexture(0);
			//dsSetColorAlpha(0.84f,0.7f,0.62f, 0.3f);
		}
	}
	else
	{
		double normal[3];
		if(!wire && tree_->GetObstacleNormal(normal))
		{
			Vector3 vNormal; matrices::arrayToVect3(normal, vNormal);
			vNormal.normalize();
			float R2[12];
            matrices::Matrix3 rotation; matrices::GetRotationMatrix(unitz, vNormal, rotation);
			//matrices::matrix3ToArray(R2, rotation);
			matrices::matrix3ToArray(R2, rotation);
			float red, green, blue;
			switch(id_)
			{
				case 1:
				{
					red = 1; green = 0; blue = 1;
					break;
				}
				case 2:
				{
					red = 0; green = 1; blue = 1;
					break;
				}
				case 3:
				{
					red = 0; green = 0; blue = 0;
					break;
				}
				default:
				{
					red = 0; green = 0; blue = 1;
					break;
				}
			}
			dsSetColorAlpha(red, green, blue,1);
			dsDrawCylinder(ps, R2, 0.05,0.05);
			dsSetColorAlpha(1.0,1.0,1.0,1);
		}
		else if(wire)
		{
			float red, green, blue;
			//if(tree_->GetObstacleNormal(normal) && color > 0)
			//{
			//	Vector3 vNormal; matrices::arrayToVect3(normal, vNormal);
			//	vNormal.normalize();
			//	//color *= Simulation::GetInstance()->simpParams_.initDir_.dot(vNormal);
			//	color = color < 0 ? 0 : color;
			//}
			if(color >= 0)
			{
				// from red to green
				red = (1 - color);
				green = color;
				blue = 0;
			}
			else
			{
				switch(id_)
				{
					case 1:
					{
						red = 1; green = 0; blue = 1;
						break;
					}
					case 2:
					{
						red = 0; green = 1; blue = 1;
						break;
					}
					case 3:
					{
						red = 0; green = 0; blue = 0;
						break;
					}
					default:
					{
						red = 0; green = 0; blue = 1;
						break;
					}
				}
			}
			dsSetColorAlpha(red, green, blue,1);
			if(tree_->GetObstacleNormal(normal))
			{
				Vector3 vNormal; matrices::arrayToVect3(normal, vNormal);
				vNormal.normalize();
				float R2[12];
                matrices::Matrix3 rotation; matrices::GetRotationMatrix(unitz, vNormal, rotation);
				//matrices::matrix3ToArray(R2, rotation);
				matrices::matrix3ToArray(R2, rotation);
				dsDrawCylinder(ps, R2, 0.05,0.05);
			}
			else
			{
				//dsDrawCylinder(ps, R, 0.03,0.03);
			}
			dsSetColorAlpha(0.84f,0.7f,0.62f, 0.8f);
		}
		else
		{
			float R[12];
			matrices::Matrix4 mr2 = matrices::Matrix4::Zero();
			mr2(0,0) = 0.6;
			mr2(1,1) = 0.6;
			mr2(2,2) = 0.5;
			mr2(3,3) = 1;
			matrixToArray(R, currentTransform * mr2);
			if(!wire || Simulation::GetInstance()->simpParams_.drawArms_)
			{
				dsDrawSphere(ps, R, effectorWidth_);
				//dsDrawCylinder(ps, R, effectorWidth_, 0.01);
			}
		}
	}

	//draw line between positions
	if(joint->GetParent())
	{
		//dsDrawLine(from, ps);
		Vector3 trans(currentTransform.block<3,1>(0,3) - vFrom);
		
		matrices::vect3ToArray(from, vFrom + (trans / 2.));
		trans.normalize();
		Matrix3 transformCube;
		GetRotationMatrix(unitz, trans, transformCube);
		matrices::Matrix3 mr2 = matrices::Matrix3::Zero();
		mr2(0,0) = 0.1;
		mr2(1,1) = 0.1;
		mr2(2,2) = 1;
		matrix3ToArray(R, transformCube * mr2);

		//dsSetTexture(1);
		//dsSetColorAlpha(0.5,0.5,0.5,1.0);
		//dsDrawCylinder(from, R, (float)(offset.norm()), 0.1);
		//dsSetTexture(0);
		//dsSetColorAlpha(0.84f,0.7f,0.62f, 0.3f);
		if(!wire || Simulation::GetInstance()->simpParams_.drawArms_)
		{
			dsDrawSphere(from, R, (float)(offset.norm() - 0.01f) / 2);
			//dsDrawCylinder(from, R, (float)(offset.norm() - 0.01f), 0.1);
		}
	}
	if(tree_->IsAnchored())
	{
		//dsSetColor(0.0,1.0,0.0);
		// draw target
		double ps[3];
		double Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
		tree_->GetTarget(ps);
		//dsDrawSphereD (ps, Identity, 0.2f);
	}
}


