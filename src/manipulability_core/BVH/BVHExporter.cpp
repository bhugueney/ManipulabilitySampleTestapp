#include "BVHExporter.h"

#include "kinematic/enums.h"
#include "MatrixDefs.h"
#include "Pi.h"
#include <Eigen/Geometry>


#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace manip_core;

namespace
{
	const matrices::Vector3 unitx(1, 0, 0);
	const matrices::Vector3 unity(0, 1, 0);
	const matrices::Vector3 unitz(0, 0, 1);
	const matrices::Vector3 unit1(sqrt(14.0)/8.0, 1.0/8.0, 7.0/8.0);
	const matrices::Vector3 zero(0,0,0);
}

struct PostureRetriever : public PostureCreatedListenerI
{
	PostureRetriever(PostureSolver& ps)
	{
		ps.RegisterPostureListener(*this);
	}
	~PostureRetriever()
	{
		// NOTHING
	}

	virtual void OnPostureCreated(NUMBER time, const RobotI* pRobot)
	{
		postures_.push_back(static_cast<const Robot*>(pRobot));
	}

	BVHExporter::T_Robots postures_;
};

struct FileHandler
{
	FileHandler(const std::string& filename)
		: filename_(filename)
		, bvhStr_()
		, tabs_("")
		, depth_(0)
	{
		// NOTHING
	}

	~FileHandler()
	{
		// NOTHING
	}

	bool Save()
	{
		ofstream myfile;
		myfile.open (filename_.c_str());
		if (myfile.is_open())
		{
			myfile << bvhStr_.rdbuf();
			myfile.close();
			return true;
		}
		return false;
	}

	const std::string AddTab(bool newLine=true)
	{
		tabs_ += "\t";
		++depth_;
		if (newLine) bvhStr_ << nl();
		return tabs_;
	}

	FileHandler& operator<< (const std::string& val)
	{
		bvhStr_ << val;
		return *this;
	}

	FileHandler& operator<< (const NUMBER& val)
	{
		bvhStr_ << val;
		return *this;
	}

	const std::string nl()
	{
		return std::string( "\n" + tabs_);
	}

	const std::string RemoveTab(bool newLine=true)
	{
		if(depth_>0)
		{
			depth_--;
			tabs_.clear();
			for(int i=0; i<depth_; ++i)
			{
				tabs_ += "\t";
			}
		}
		if (newLine) bvhStr_ << nl();
		return tabs_;
	}

	std::stringstream bvhStr_; // To write the new bvh
	const std::string filename_;
	std::string tabs_;
	int depth_;
};

struct ExporterPImpl
{
	ExporterPImpl()
	{
		// NOTHING
	}

	~ExporterPImpl()
	{
		// NOTHING
	}

	void WriteOffsetLine(FileHandler& f, const matrices::Vector3& offset) const
	{
		// going from z up to y up
		f << "OFFSET\t" << offset(0) << "\t" << offset(2) << "\t" << -offset(1);
	}

	// true if endEffector
	bool WriteJointOffset(FileHandler& f, const Joint& joint, std::string name ="", const matrices::Vector3& correction = zero) const
	{
		matrices::Vector3 offset = joint.GetR() - correction;
		f << f.nl() << "JOINT " << name << f.nl() << "{";
		f.AddTab();
			WriteOffsetLine(f, offset*10.);
			f << f.nl() << "CHANNELS 3 Zrotation Xrotation Yrotation";
		if(joint.IsEffector())
		{
			f << f.nl() << "End Site" << f.nl() << "{";
			f.AddTab();
				WriteOffsetLine(f, offset);
			f.RemoveTab();				
			f << "}";
			return true;
		}
		return false;
	}
		
	bool ExportJoint(const Joint& joint, const std::string& treeName, FileHandler& f, int id, const matrices::Vector3& correction = zero) const
	{
		matrices::Vector3 offset = joint.GetR();
		if( offset != zero ) // each joint rotates only around one axis in my representation
		{
			stringstream ss; ss << id;
			if(!WriteJointOffset(f, joint, treeName + ss.str(), correction))
			{
				if(joint.pChild_)
				{
					ExportJoint( *joint.pChild_, treeName, f, ++id);
				}
			}
			f.RemoveTab();
			f << "}";
		}
		else if(joint.pChild_)
		{
			ExportJoint( *joint.pChild_, treeName, f, id);
		}
		return true;
	}

	bool ExportTree(const Tree& tree, FileHandler& f, const matrices::Vector3& correction) const
	{
		//TODO add missing joint
		ExportJoint((*tree.GetRoot()), manip_core::enums::StringTreeType(tree.GetTreeType()), f, 0, correction);
		return true;
	}

	bool ExportHierarchyWriteOffset(FileHandler& f, Robot::T_Hierarchy::const_iterator it0, const Robot::T_Hierarchy& hierarchy, const Joint* currentJoint, const matrices::Vector3& correction) const
	{
		if(it0 == hierarchy.end())
		{
			return true;
		}
		else
		{
			matrices::Vector3 offset(currentJoint->GetR());
			if(!WriteJointOffset(f, *currentJoint, "TRONC"))
			{
				for(Robot::T_TreeCIT it = it0->begin(); it!= it0->end(); ++it)
				{
					ExportTree((*(*it)), f, currentJoint->GetR() + correction);
				}
				ExportHierarchyWriteOffset(f, ++it0, hierarchy, currentJoint->pChild_, currentJoint->GetR() + correction);
			}
			f.RemoveTab();
			f << "}";
			return true;
		}
	}

	bool ExportHierarchy(FileHandler& f, const Robot::T_Hierarchy& hierarchy, const Joint* currentJoint) const
	{
		Robot::T_Hierarchy::const_iterator it0 = hierarchy.begin();
		for(Robot::T_TreeCIT it = it0->begin(); it!= it0->end(); ++it)
		{
			ExportTree((*(*it)), f, currentJoint->GetR());
		}
		ExportHierarchyWriteOffset(f, ++it0, hierarchy, currentJoint->pChild_, currentJoint->GetR());
		return true;
	}

	bool ExportRobotStructure(const Robot& robot, FileHandler& f) const
	{
		Joint* currentJoint = robot.GetTorso()->GetRoot();
		assert(currentJoint);
		matrices::Vector3 offset(currentJoint->GetR());
		f << "HIERARCHY" << f.nl() << "ROOT " << manip_core::enums::StringRobotType(robot.RobotType()) << f.nl() << "{" ;
		f.AddTab();
		// TODO handle rotation matrix, start with everything at 0 for now
			WriteOffsetLine(f, offset*10.);
			f << f.nl() << "CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation";
			const Robot::T_Tree trees(robot.GetTrees());
			const Robot::T_Hierarchy hierarchy(robot.GetHierarchy());
			ExportHierarchy(f, hierarchy, currentJoint);
		f.RemoveTab();
		f << "}";
		// TODO handle rotation matrix, start with everything at 0 for now
		return true;
	}

	bool ExportMotion(const BVHExporter::T_Robots& postures, FileHandler& f) const
	{
		f << f.nl() << "MOTION\nFrames:\t" << (NUMBER)(postures.size()) << "\nFrame Time: 1\n";
		for(BVHExporter::T_Robots::const_iterator it = postures.begin(); it != postures.end(); ++ it)
		{
			ExportRobotMotion(*(*it), f);
			f << f.nl();
		}
		return true;
	}

	void GetRotation(const Joint* joint, matrices::Vector3& res) const
	{
		// remember , going from z up to y up
		matrices::Vector3 axis = joint->GetRotationAxis();
		NUMBER angle = joint->GetTheta() * RadiansToDegrees;
		if(axis == unitx)
		{
			res(0) = angle;
		}
		else if(axis == unity)
		{
			res(2) = -angle;
		}
		else
		{
			res(1) = angle;
		}
	}

	bool ExportJointMotion(const Joint& joint, FileHandler& f) const
	{
		matrices::Vector3 res(0,0,0);
		const Joint * j = &joint;
		do
		{
			GetRotation(j, res);
			j = j->pChild_;
		}while(j && j->GetR() == zero);
		f << res(2) << "\t" << res(0)<< "\t"  << res(1)<< "\t" ;
		if(j)
		{
			return ExportJointMotion(*j, f);
		}
		return true;
	}

	bool ExportTreeMotion(const Tree& tree, FileHandler& f) const
	{
		//TODO add missing joint
		ExportJointMotion(*tree.GetRoot(),f);
		return true;
	}

	bool ExportRobotMotion(const Robot& robot, FileHandler& f) const
	{
		const Robot::T_Hierarchy hierarchy(robot.GetHierarchy());
		// TODO for the time being just put 0 in there
		matrices::Vector3 translation = robot.ToWorldCoordinates().block(0,3,3,1) * 10.;
		f << translation(0) << "\t" << translation(2) << "\t" << -translation(1) << "\t";
		//vR = vYXZ
		matrices::Matrix3 angleMat = robot.ToWorldCoordinates().block(0,0,3,3);
		matrices::Vector3 angles = angleMat.eulerAngles(1,0,2) * RadiansToDegrees;
		//f << angles(2) << "\t" << angles(0) << "\t" << angles(1) << "\t";
		//f << angles(2) << "\t" << angles(1) << "\t" << angles(0) << "\t";
		f << -angles(0) << "\t" << angles(1) << "\t" << angles(2) << "\t";
		for(Robot::T_Hierarchy::const_iterator it0 = hierarchy.begin(); it0!= hierarchy.end(); ++it0)
		{
			if(it0!=hierarchy.begin())
			{
				f << "0\t0\t0\t";
			}
			for(Robot::T_TreeCIT it = it0->begin(); it!= it0->end(); ++it)
			{
				ExportTreeMotion((*(*it)), f);
			}
		}
		f << f.nl();
		return true;
	}

};


BVHExporter::BVHExporter()
	: pImpl_(new ExporterPImpl())
{
	// NOTHING
}

BVHExporter::~BVHExporter()
{
	// NOTHING
}

bool BVHExporter::ExportToBVH(const T_Robots& postures, const std::string& filename) const
{
	// TODO
	if(postures.size() < 0)
	{
		return false;
	}
	FileHandler handler(filename);
	pImpl_->ExportRobotStructure(*(postures[0]), handler);
	pImpl_->ExportMotion(postures, handler);
	return handler.Save();
}

bool BVHExporter::ExportToBVH(PostureSolver& postureSolver, const std::string& filename) const
{
	PostureRetriever psr(postureSolver);
	return ExportToBVH(psr.postures_, filename);
}




// TESTS

//#include "kinematic/RobotFactory.h"
//
//using namespace matrices;
//
//int main(int argc, char *argv[])
//{
//	BVHExporter::T_Robots postures;
//	factories::RobotFactory robotfact_;
//	//Matrix4 robotBasis(Rotz4(0.8));
//	//Matrix4 robotBasis(Rotx4(0.8));
//	Matrix4 robotBasis(Rotx4(0.8));
//	//Robot * pRobot = robotfact_.CreateRobot(manip_core::enumsHuman, robotBasis);
//	Robot * pRobot = robotfact_.CreateRobot(manip_core::enumsQuadruped, robotBasis);
//	postures.push_back(pRobot);
//	BVHExporter exporter;
//	exporter.ExportToBVH(postures, "../out/bvh/test.bvh");
//	return 0;
//}