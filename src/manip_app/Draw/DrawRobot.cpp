
#include "DrawRobot.h"
#include "DrawTree.h"
#include "DrawSamples.h"
//#include "DrawSupportPolygon.h"

#include "API/RobotI.h"
#include "API/TreeI.h"

#include "MatrixDefs.h"

#include "Simulation.h"
#ifdef WIN32
#include <windows.h>
#endif
#include "MouseTrack.h"
#include "MainTools.h"
#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include <vector>

using namespace std;
using namespace matrices;
using namespace manip_core;

struct PImpl
{
	PImpl(const RobotI* robot)
		:robot_(robot)
		,drawSupport_(false)
	{
		/* UNCOMMENT FOR DISPLAY
	    for (unsigned int i = 0; i< robot_->GetNumTrees(); ++i)
		{
			drawSamples_.push_back(new DrawSamples(robot_, robot->GetTreeI(i), i, true));
		}*/
	}

	~PImpl()
	{
		for (std::vector<DrawSamples*>::iterator it = drawSamples_.begin(); it != drawSamples_.end(); ++it)
		{
			delete (*it);
		}
	}

	const RobotI* robot_;

	typedef vector<DrawTree> T_Tree;
	typedef T_Tree::iterator T_TreeIT;
	typedef T_Tree::const_iterator T_TreeCIT;
	T_Tree drawTrees_;
	std::vector<DrawSamples*> drawSamples_;
	bool drawSupport_;
};



//TODO listener in case of adding new trees
DrawRobot::DrawRobot(const RobotI* robot)
	: pImpl_(new PImpl(robot))
{
	const TreeI* trunk = robot->GetTorsoI();

	if(trunk)
	{
		pImpl_->drawTrees_.push_back(DrawTree(trunk, 0.04f, 0.25f));
	}
	int nTrees = robot->GetNumTrees();
	double vect[3];
	for(int i=0; i < nTrees; ++i)
	{
		const TreeI* pTree = robot->GetTreeI(i);
		robot->GetTreeAttach(i, vect);
		pImpl_->drawTrees_.push_back(DrawTree(pTree, vect));
		//pImpl_->drawTrees_.push_back(DrawTree(pTree, vect,0.02f, 0.05f, i));
	}
	/*robot.Accept(*this);
	const Tree* trunk = robot.GetTorso();
	if(trunk)
	{
		pImpl_->drawTrees_.push_back(DrawTree(*trunk));
	}*/
}

DrawRobot::~DrawRobot()
{
	// TODO
}

void DrawRobot::ToggleSupportPolygon(bool onoff)
{
	pImpl_->drawSupport_ = onoff;
}

const manip_core::RobotI* DrawRobot::GetRobot()
{
	return pImpl_->robot_;
}


#include "Pi.h"
namespace
{
	
	static void setTransform (const float pos[3], const float R[12])
	{
	  GLfloat matrix[16];
	  matrix[0]=R[0];
	  matrix[1]=R[4];
	  matrix[2]=R[8];
	  matrix[3]=0;
	  matrix[4]=R[1];
	  matrix[5]=R[5];
	  matrix[6]=R[9];
	  matrix[7]=0;
	  matrix[8]=R[2];
	  matrix[9]=R[6];
	  matrix[10]=R[10];
	  matrix[11]=0;
	  matrix[12]=pos[0];
	  matrix[13]=pos[1];
	  matrix[14]=pos[2];
	  matrix[15]=1;
	  glPushMatrix();
	  glMultMatrixf (matrix);
	}

	void DrawEllipse(const RobotI* robot, int treeId)
	{
	//if(treeId != 2) return ;
		double robotCoord[16];
		Matrix4 currentTransform;
		Vector3 pos, v1, v2, v3;
		double from[3];
		double u1[3];
		double u2[3];
		double u3[3];
		double sig1, sig2, sig3;
		TreeI* tree = robot->GetTreeI(treeId);
		if(tree)
		{
			robot->ToWorldCoordinates(robotCoord);
			matrices::array16ToMatrix4(robotCoord, currentTransform);
			tree->EndEffectorPosition(from);
			matrices::arrayToVect3(from, pos);
			pos = matrices::matrix4TimesVect3(currentTransform, pos);
			matrices::vect3ToArray(from, pos);
			tree->GetEllipsoidAxes(u1, u2, u3, sig1, sig2, sig3);

			matrices::arrayToVect3(u1, v1);
			matrices::arrayToVect3(u2, v2); 
			matrices::arrayToVect3(u3, v3);

			//Getting svd
			/*Matrix3d u = tree->svd_.matrixU();
			VectorXd sing = tree->svd_.singularValues();
			DrawEllipse( sing(0), sing(1), sing(2));*/
			/*const Vector3 from( -1, 0, 2 );
			/*DrawArrow( from, u.col(0) );
			DrawArrow( from, u.col(1) );
			DrawArrow( from, u.col(2) );*/

			float rx, ry, rz, xf, yf, zf;
			rx = sig1;
			ry = sig2;
			rz = sig3;

			float sina;
			float a, b, x, y, z;

			float pas = 0.025f;
			glColor3f(0.3f, 0.4f, 0.7f);
	
			glBegin(GL_POINTS);


			//directing vectors values
			float ox, oy, oz, x0,y0,z0,x1,y1,z1,x2,y2,z2;
			x0 = v1(0); x1 = v2(0); x2 = v3(0);
			y0 = v1(1); y1 = v2(1); y2 = v3(1);
			z0 = v1(2); z1 = v2(2); z2 = v3(2);
			ox = pos(0); oy = pos(1); oz = pos(2);

			for (a=0.0f; a <= Pi; a+=pas)
			{
				z = rz*cosf(a);
				sina = sinf (a);
				for (b=-Pi; b <= Pi; b+=pas)
				{			
			  	  
					x =rx*sina * cosf(b);
		  
					y =ry*sina * sinf(b);
	  
					//normal(x, y, z);
					//rotation with correct angles
					xf = ox + x * x0 + y * x1 + z * x2;
					yf = oy + x * y0 + y * y1 + z * y2;
					zf = oz + x * z0 + y * z1 + z * z2;

					//glVertex3f(x,y,z) ;
					glVertex3f(xf,yf,zf) ;

				}
			}
			glEnd();
		}
	}
}

void DrawRobot::DrawEllipsoid(int treeId) const
{
	//if(treeId != 2) return ;
	const Vector3 unitz(0,0,1);
	double robotCoord[16];
	Matrix4 currentTransform;
	Vector3 pos, v1, v2, v3;
	double from[3];
	double u1[3];
	double u2[3];
	double u3[3];
	TreeI* tree = pImpl_->robot_->GetTreeI(treeId);
	if(tree)
	{

		/*pImpl_->robot_->ToWorldCoordinates(robotCoord);
		matrices::array16ToMatrix4(robotCoord, currentTransform);
		tree->EndEffectorPosition(from);
		matrices::arrayToVect3(from, pos);
		pos = matrices::matrix4TimesVect3(currentTransform, pos);
		matrices::vect3ToArray(from, pos);
		tree->GetEllipsoidAxes(u1, u2, u3);

		Vector3 u1b, u2b, u3b;  matrices::arrayToVect3(u1, u1b);
		u1b.normalize();
		Matrix3 transformCube;
		GetRotationMatrix(unitz, u1b, transformCube);

		matrices::arrayToVect3(u1, u1b);
		matrices::arrayToVect3(u2, u2b);
		matrices::arrayToVect3(u3, u3b);


		Vector3 x, y, z;
		matrices::arrayToVect3(u1, u1b);
		matrices::arrayToVect3(u2, u2b);
		matrices::arrayToVect3(u3, u3b);
		double R[12];

		matrices::Matrix3 mr2 = matrices::Matrix3::Zero();
		mr2(0,0) = u2b.norm();
		mr2(1,1) = u3b.norm();
		mr2(2,2) = u1b.norm();
		dsSetColor(	0.3f, 0.4f, 0.7f);
		matrix3ToArrayD(R, transformCube * mr2);
		dsDrawSphereD(from, R, 1);*/

		pImpl_->robot_->ToWorldCoordinates(robotCoord);
		matrices::array16ToMatrix4(robotCoord, currentTransform);
		tree->EndEffectorPosition(from);
		matrices::arrayToVect3(from, pos);
		pos = matrices::matrix4TimesVect3(currentTransform, pos);
		matrices::vect3ToArray(from, pos);
		tree->GetEllipsoidAxes(u1, u2, u3);

		matrices::arrayToVect3(u1, v1);
		matrices::arrayToVect3(u2, v2); 
		matrices::arrayToVect3(u3, v3);
		//glColor3f(0.3f, 0.4f, 0.7f);
		dsSetColor(	0.3f, 0.4f, 0.7f);
		//dsSetTexture(0);
		//dsSetColorAlpha(0,0,1,0.3);
		//glPushMatrix();
		//// translating to point center
		//glTranslatef(pos.x(), pos.y(), pos.z());
		////glRotate(angle, axis.x, axis.y, axis.z);
		//// Scaling to ellipsoid values
		//glScalef(v1.norm(), v2.norm(), v3.norm());
		//gluSphere(quad, 1, 100, 100);
		//glPopMatrix();
		v1 = pos + v1; matrices::vect3ToArray(u1, v1);
		v2 = pos + v2; matrices::vect3ToArray(u2, v2);
		v3 = pos + v3; matrices::vect3ToArray(u3, v3);
		dsSetColor(	1.f, 0, 0);
		dsDrawLineD(from, u1);
		dsSetColor(	0, 1, 0);
		dsDrawLineD(from, u2);
		dsSetColor(	0, 0, 1);
		dsDrawLineD(from, u3);
	}
}

void DrawRobot::Draw(bool transparency) const
{
	dsSetTexture(0);
	double currentTransform[16];
	pImpl_->robot_->ToWorldCoordinates(currentTransform);
	Matrix4 transform;
	matrices::array16ToMatrix4(currentTransform, transform);
	int i = 0;
	for(PImpl::T_TreeCIT it = pImpl_->drawTrees_.begin(); it!= pImpl_->drawTrees_.end(); ++it, ++i)
	{
		it->Draw(transform, transparency);
		if(Simulation::GetInstance()->simpParams_.drawEllipseAxes_ )
			DrawEllipsoid(i);
		if(Simulation::GetInstance()->simpParams_.drawEllipsoid_ )
			DrawEllipse(pImpl_->robot_, i);
	}
	if(pImpl_->robot_->GetType() == manip_core::enums::robot::SpiderRace 
		|| pImpl_->robot_->GetType() == manip_core::enums::robot::SpiderSix)
	{
		float R[12];
		float ps[3];
		matrices::vect3ToArray(ps, matrices::matrix4TimesVect3(transform, Vector3(-0.3, 0, 0)));
		matrices::Matrix4 mr2 = matrices::Matrix4::Zero();
		mr2(0,0) = 1;
		mr2(1,1) = 0.5;
		mr2(2,2) = 0.7;
		mr2(3,3) = 1;
		matrixToArray(R, transform * mr2);
		dsDrawSphere(ps, R, 0.3);
		mr2(0,0) = 0.6;
		mr2(1,1) = 0.5;
		mr2(2,2) = 0.5;
		mr2(3,3) = 1;
		matrixToArray(R, transform * mr2);
		matrices::vect3ToArray(ps, matrices::matrix4TimesVect3(transform, Vector3(+0.3, 0, 0)));
		dsDrawSphere(ps, R, 0.2);
	}
	dsSetTexture(0);

	double com[3];
	pImpl_->robot_->ComputeCom(com);
	Vector3 vCom(com[0], com[1], com[2]);
	vCom = matrices::matrix4TimesVect3(transform, vCom);
	com[0] = vCom(0);
	com[1] = vCom(1);
	com[2] = vCom(2);
	double Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	//dsDrawSphereD(com, Identity, 0.1f);

	//if(pImpl_->drawSupport_)
	//{
	//	Vector3 com = matrices::matrix4TimesVect3( pImpl_->robot_->ToWorldCoordinates(), pImpl_->robot_->ComputeCom());
	////Vector3 com = pRobot->ComputeCom();
	//	float Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	//	float t[3];
	//	t[0] = (float)(com(0));
	//	t[1] = (float)(com(1));
	//	t[2] = (float)(com(2));
	//	dsDrawSphere(t, Identity, 0.1f);
	//	SupportPolygon sp(*pImpl_->robot_);
	//	DrawSupportPolygon dsp(*pImpl_->robot_, sp);
	//	dsp.Draw();
	//}
	if(Simulation::GetInstance()->simpParams_.drawSamples_)
	{
		if (Simulation::GetInstance()->simpParams_.drawSamplesByOne_)
		{
			if(pImpl_->drawSamples_.size() > 0)
			{
				if(Simulation::GetInstance()->simpParams_.currentSample_ > (pImpl_->drawSamples_.size() -1))
				{
					Simulation::GetInstance()->simpParams_.currentSample_ = 0;
				}
				pImpl_->drawSamples_[Simulation::GetInstance()->simpParams_.currentSample_]->Draw(transform, true);
			}
		}
		else
		{
			for (std::vector<DrawSamples*>::iterator it = pImpl_->drawSamples_.begin(); it != pImpl_->drawSamples_.end(); ++it)
			{
				(*it)->Draw(transform, true);
			}
		}
	}
}

void DrawRobot::DrawNoTexture() const
{
	dsSetTexture(0);
	double currentTransform[16];
	pImpl_->robot_->ToWorldCoordinates(currentTransform);
	Matrix4 transform;
	matrices::array16ToMatrix4(currentTransform, transform);
	for(PImpl::T_TreeCIT it = pImpl_->drawTrees_.begin(); it!= pImpl_->drawTrees_.end(); ++it)
	{
		it->DrawNoTexture(transform);
	}
	dsSetTexture(0);

	//if(pImpl_->drawSupport_)
	//{
	//	Vector3 com = matrices::matrix4TimesVect3( pImpl_->robot_->ToWorldCoordinates(), pImpl_->robot_->ComputeCom());
	////Vector3 com = pRobot->ComputeCom();
	//	float Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	//	float t[3];
	//	t[0] = (float)(com(0));
	//	t[1] = (float)(com(1));
	//	t[2] = (float)(com(2));
	//	dsDrawSphere(t, Identity, 0.1f);
	//	SupportPolygon sp(*pImpl_->robot_);
	//	DrawSupportPolygon dsp(*pImpl_->robot_, sp);
	//	dsp.Draw();
	//}
}

//void DrawRobot::Visit(const Tree& tree, const Joint* anchor)
//{
//	pImpl_->drawTrees_.push_back(DrawTree(tree));
//}

