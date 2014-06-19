#ifndef  _BVHEXPORTER_H_
#define  _BVHEXPORTER_H_

#include "kinematic/Robot.h"
#include "kinematic/Tree.h"
#include "kinematic/Joint.h"
#include "posture/PostureSolver.h"
#include <memory>
#include <string>

/**
*  \class BVHExporter
*  \brief Export A robot to a readable BVH file format
*/

struct ExporterPImpl;

class  BVHExporter
{
public:
	 BVHExporter();
	~BVHExporter();
	
private:
	BVHExporter(const BVHExporter&);
	BVHExporter& operator=(const BVHExporter&);

public:
	typedef std::vector<const Robot*>	T_Robots;

public:
	bool ExportToBVH(const T_Robots& /*postures*/	 , const std::string& /*filename*/) const;
	bool ExportToBVH(PostureSolver& /*postureSolver*/, const std::string& /*filename*/) const;

private:
	std::auto_ptr<ExporterPImpl> pImpl_;
};


#endif // _BVHEXPORTER_H_
