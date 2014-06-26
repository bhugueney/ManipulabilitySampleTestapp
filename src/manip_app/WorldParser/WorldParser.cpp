#include "WorldParser.h"
#include "WorldParserObj.h"
#include "spline/exact_cubic.h"


#include "Simulation.h"

#include "MatrixDefs.h"

#include "Pi.h"

using namespace matrices;

WorldParser::WorldParser()
	:manager_(Simulation::GetInstance()->manager_)
{
	// NOTHING
}

WorldParser::~WorldParser()
{
	//deleted by sim
	/*for(int i=0; i < values_.size(); ++i)
	{
		delete values_[i];
	}*/
}

/*<world>
  <robot type="0" x="-8" y="1" z="3" rotx="0" roty="0" rotz="0">
	<tree id="0"> optional
		<joint id="0" angle="0"/>
		<joint id="1" angle="0"/>
		<joint id="2" angle="0"/>
		<joint id="3" angle="0"/>
	</tree>
  </robot>
  <obstacles>
	<obstacle> 
		<p0 x="" y="" z=""/>
		<p1 x="" y="" z=""/>
		<p2 x="" y="" z=""/>
		<p3 x="" y="" z=""/>
	</obstacle>
	...
	<obstacle> 
		<p0 x="" y="" z=""/>
		<p1 x="" y="" z=""/>
		<p2 x="" y="" z=""/>
		<p3 x="" y="" z=""/>
	</obstacle>
	...
	<verticalChess> 
		<depth x="" y="" z=""/>
		<p0 x="" y="" z=""/>
		<p3 x="" y="" z=""/>
	</verticalChess>
	...
	<prise> 
		<p0 x="" y="" z=""/>
	</prise>
  </obstacles>
</world>*/

void WorldParser::InitWorld(const std::string& filename){
	std::string file;
	xml::xifstream xis(filename);
	xis >> xml::start("program")
		>> xml::start("world")
		>> xml::attribute("file", file)
		>> xml::end
		>> xml::end;
	CreateWorld(file);
}

void WorldParser::CreateWorld(const std::string& filename)
{
	bool modePlanif = false;
	int type; int nbSamples = 10000;
	float iDirx = 1;
	float iDiry = 0;
	float iDirz = 0;
	float backr = 1;
	float backg = 1;
	float backb = 1;
	float x, y, z, rx, ry, rz;
	float cx, cy, cz, crx, cry, crz;
	float speed = 1/4;
	std::string camType = "";
	std::string robotFile = "";
	bool ground(false);
	bool gait(false);
	bool autoRotate(false);
	bool autoRotateLeg(false);
	bool jumpToTarget(false);
	bool drawSplines(false);
	bool drawNormals(false);
	bool rotatewithspline(false);
	bool reachCom(false);
	bool humanrotate(false);
	bool rotatewithsplinedir(false);
	std::vector<std::pair<double, Vector3> > splinePoints;
	std::string obj = "";

	xml::xifstream xis( filename );

	xis >> xml::start( "world" )
			>> xml::optional 
				>> xml::start( "startpaused" )
				>> xml::attribute( "val", Simulation::GetInstance()->simpParams_.pause_ )
				>> xml::end
			>> xml::start( "robot" )
				>> xml::optional 
				>> xml::attribute( "type", type )
				>> xml::optional 
				>> xml::attribute( "robotFile", robotFile )
				>> xml::optional 
				>> xml::attribute( "samples", nbSamples )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
				>> xml::attribute( "rotx", rx )
				>> xml::attribute( "roty", ry )
				>> xml::attribute( "rotz", rz )
				>> xml::optional 
				>> xml::attribute( "iDirx", iDirx )
				>> xml::optional 
				>> xml::attribute( "iDiry", iDiry )
				>> xml::optional 
				>> xml::attribute( "iDirz", iDirz )
				>> xml::optional 
				>> xml::attribute( "gait", gait )
				>> xml::optional 
				>> xml::attribute( "jointlimit", Simulation::GetInstance()->simpParams_.joint_ )
				>> xml::optional 
				>> xml::attribute( "autorotate", autoRotate )
				>> xml::optional 
				>> xml::attribute( "autorotateleg", autoRotateLeg )
				>> xml::optional 
				>> xml::attribute( "jumptotarget", jumpToTarget )
				>> xml::optional 
				>> xml::attribute( "drawsplines", drawSplines )
				>> xml::optional 
				>> xml::attribute( "drawEllipseAxes", Simulation::GetInstance()->simpParams_.drawEllipseAxes_ )
				>> xml::optional 
				>> xml::attribute( "drawEllipsoid", Simulation::GetInstance()->simpParams_.drawEllipsoid_ )
				>> xml::optional 
				>> xml::attribute( "drawnormals", drawNormals )
				>> xml::optional 
				>> xml::attribute( "rotatewithspline", rotatewithspline )
				>> xml::optional 
				>> xml::attribute( "rotatewithsplinedir", rotatewithsplinedir )
				>> xml::optional 
				>> xml::attribute( "reachcom", reachCom )
				>> xml::optional 
				>> xml::attribute( "reachcomy", Simulation::GetInstance()->simpParams_.reachComY_ )
				>> xml::optional 
				>> xml::attribute( "reachcomrotate", Simulation::GetInstance()->simpParams_.reachComRotate_ )
				>> xml::optional 
				>> xml::attribute( "humanrotate", humanrotate )
					>> xml::optional 
					>> xml::list(*this, &WorldParser::ConfigureTree)
				>> xml::optional 
				>> xml::attribute( "speed", speed )
				>> xml::optional 
				>> xml::attribute( "planif", Simulation::GetInstance()->simpParams_.planif_ )
			>> xml::end
			>> xml::optional
				>> xml::start( "rootTrajectory" )
						>> xml::optional 
						>> xml::attribute( "inittimer", Simulation::GetInstance()->simpParams_.initTimer_ )
						>> xml::list(*this, &WorldParser::AddRootTrajectoryPoint, splinePoints)
				>> xml::end
			>> xml::optional 
				>> xml::start( "obj" )
					>> xml::attribute( "name", obj )
					>> xml::optional 
					>> xml::attribute( "ground", ground )
				>> xml::end
			>> xml::optional
				>> xml::start( "camera" )					
					>> xml::attribute( "x", cx )
					>> xml::attribute( "y", cy )
					>> xml::attribute( "z", cz )
					>> xml::attribute( "rotx", crx )
					>> xml::attribute( "roty", cry )
					>> xml::attribute( "rotz", crz )
					>> xml::optional 
					>> xml::attribute( "type", camType )
				>> xml::end
			>> xml::optional 
				>> xml::start( "light" )					
					>> xml::attribute( "x", Simulation::GetInstance()->simpParams_.lightX )
					>> xml::attribute( "y", Simulation::GetInstance()->simpParams_.lightY )
					>> xml::attribute( "shadows", Simulation::GetInstance()->simpParams_.drawShadows_ )
				>> xml::end
			>> xml::optional 
				>> xml::start( "background" )
					>> xml::attribute( "r", backr )
					>> xml::attribute( "g", backg )
					>> xml::attribute( "b", backb )
					>> xml::attribute( "sky", Simulation::GetInstance()->simpParams_.drawSky_ )
					>> xml::attribute( "ground", Simulation::GetInstance()->simpParams_.drawGround_ )
				>> xml::end
			>> xml::start( "obstacles" )
				>> xml::optional 
				>> xml::attribute( "offSet", Simulation::GetInstance()->simpParams_.obstacleOffset_ )
				>> xml::list(*this, &WorldParser::CreateObstacle)
			>> xml::end;
				
	Simulation::GetInstance()->simpParams_.obstacleOffset_ = false;
	if(obj != "")
	{
		WorldParserObj objParse;
		objParse.CreateWorld(obj, ground);
	}
	Camera_ABC * camera;
	if(camType == "follow")
	{
		camera = new CameraFollow(matrices::Vector3(cx, cy, cz), matrices::Vector3(crx, cry, crz));
	}
	else
	{
		camera = new Camera_ABC(matrices::Vector3(cx, cy, cz), matrices::Vector3(crx, cry, crz));
	}
	Simulation::GetInstance()->simpParams_.SetCamera(camera);
	Simulation::GetInstance()->simpParams_.initDir_ = Vector3(iDirx, iDiry, iDirz);
	Simulation::GetInstance()->simpParams_.autorotate_ = autoRotate;
	Simulation::GetInstance()->simpParams_.autorotateleg_ = !autoRotate && autoRotateLeg;
	Simulation::GetInstance()->simpParams_.jumpToTarget_ = jumpToTarget;
	Simulation::GetInstance()->simpParams_.drawSplines_ = drawSplines;
	Simulation::GetInstance()->simpParams_.drawNormals_ = drawNormals;
	Simulation::GetInstance()->simpParams_.reachCom_ = reachCom;
	Simulation::GetInstance()->simpParams_.nbSamples_ = nbSamples;
	Simulation::GetInstance()->simpParams_.speed_ = speed;
	Simulation::GetInstance()->simpParams_.gait_ = gait;
	Simulation::GetInstance()->simpParams_.rotatewithspline_ = rotatewithspline;
	Simulation::GetInstance()->simpParams_.rotatewithsplinedir_ = rotatewithsplinedir;
	Simulation::GetInstance()->simpParams_.humanrotate_ = humanrotate;
	Simulation::GetInstance()->simpParams_.background_ = Vector3(backr, backg, backb);
	if(splinePoints.size() > 1)
	{
		if(Simulation::GetInstance()->simpParams_.planif_)
		{
			Simulation::GetInstance()->simpParams_.rootTrajectory_ = false;
			spline::exact_cubic<> tg (splinePoints.begin(), splinePoints.end());
			float delta = (tg.max() - tg.min()) / 50;
			for(float it = tg.min(); it < tg.max(); it = it + delta)
			{
				manager_.GetPostureManager()->AddCheckPoint(it, tg(it));
			}
		}
		else
		{
			Simulation::GetInstance()->simpParams_.rootTrajectory_ = true;
			Simulation::GetInstance()->simpParams_.rootSpline = new spline::exact_cubic<>(splinePoints.begin(), splinePoints.end());
		}
	}
	else
	{
		Simulation::GetInstance()->simpParams_.rootTrajectory_ = false;
	}
	if(robotFile.length() > 0)
	{
		CreateRobot(robotFile, x, y, z, rx, ry, rz);
	}
	else
	{
		CreateRobot(type, x, y, z, rx, ry, rz);
	}
	manager_.Initialize(true);
}

void WorldParser::CreateRobot(const int type, const float x   , const float y   , const float z
											, const float rotx, const float roty, const float rotz) const
{
	matrices::Matrix4 basis(MatrixX::Identity(4,4));
	if(rotx != 0)
	{
		basis = basis * Rotx4(DegreesToRadians * rotx);
	}
	if(roty != 0)
	{
		basis.block(0,0,3,3) = basis.block(0,0,3,3) * Roty3(DegreesToRadians * roty);
	}
	if(rotz != 0)
	{
		basis = basis * Rotz4(DegreesToRadians * rotz);
	}
	basis(0,3) = x;
	basis(1,3) = y;
	basis(2,3) = z;
	Simulation::GetInstance()->simpParams_.robotType_ = (manip_core::enums::robot::eRobots)(type);
	Simulation::GetInstance()->simpParams_.robotBasis_ = basis;
	Simulation::GetInstance()->simpParams_.angleValues_ = values_;
}

//#include "kinematics\joint_io.h"

void WorldParser::CreateRobot(const std::string& robotFile, const float x   , const float y   , const float z
											, const float rotx, const float roty, const float rotz) const
{
    /*manip_core::joint_def_t* root = kinematics::ReadTree<float, float, 3, 5, false>(robotFile);
	matrices::Matrix4 basis(MatrixX::Identity(4,4));
	if(rotx != 0)
	{
		basis = basis * Rotx4(DegreesToRadians * rotx);
	}
	if(roty != 0)
	{
		basis.block(0,0,3,3) = basis.block(0,0,3,3) * Roty3(DegreesToRadians * roty);
	}
	if(rotz != 0)
	{
		basis = basis * Rotz4(DegreesToRadians * rotz);
	}
	basis(0,3) = x;
	basis(1,3) = y;
	basis(2,3) = z;
	Simulation::GetInstance()->simpParams_.root = root;
	Simulation::GetInstance()->simpParams_.robotBasis_ = basis;
    Simulation::GetInstance()->simpParams_.angleValues_ = values_;*/
}

void WorldParser::ConfigureTree(const std::string& name, xml::xistream& xis)
{
	int id;bool lock = false;
	if(name == "tree")
	{
		xis >> xml::attribute( "id", id )
			>> xml::optional 
			>> xml::attribute( "lock", lock )
			>> xml::list(*this, &WorldParser::ConfigureTreeJoint, id);
		if(lock)
			Simulation::GetInstance()->simpParams_.locked_.push_back(id);
	}
}

void WorldParser::ConfigureTreeJoint(const std::string& name, xml::xistream& xis, int tree)
{
	int id; double val;
	if(name == "joint")
	{
		xis >> xml::attribute( "id", id )
			>> xml::attribute( "angle", val );
		
		if(tree<values_.size())
		{
			values_[tree].push_back( val * DegreesToRadians);
		}
		else
		{
			T_JointValues vals;
			vals.push_back(val * DegreesToRadians);
			values_.push_back(vals);
		}
	}

}

void WorldParser::AddRootTrajectoryPoint(const std::string& name, xml::xistream& xis, T_Waypoints& splinePoints) const
{
	double x, y, z, t;
	if(name == "waypoint")
	{
		xis >> xml::attribute( "x", x )
			>> xml::attribute( "y", y )
			>> xml::attribute( "z", z )
			>> xml::attribute( "t", t );
	}
	t = splinePoints.size() ;
	splinePoints.push_back(std::make_pair(t, Vector3(x, y, z)));
}

void WorldParser::CreateObstacle(const std::string& name, xml::xistream& xis) const
{
	matrices::Vector3 p0, p1, p2, p3;
	float x, y, z;
	if(name == "obstacle")
	{
		xis >> xml::start( "p0" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p0 = Vector3(x, y, z);
		xis >> xml::start( "p1" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p1 = Vector3(x, y, z);
		xis >> xml::start( "p2" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p2 = Vector3(x, y, z);
		xis >> xml::start( "p3" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p3 = Vector3(x, y, z);

		manager_.AddObstacle(p0, p1, p2, p3);
	}
	if(name == "ground")
	{
		xis >> xml::start( "p0" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p0 = Vector3(x, y, z);
		xis >> xml::start( "p1" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p1 = Vector3(x, y, z);
		xis >> xml::start( "p2" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p2 = Vector3(x, y, z);
		xis >> xml::start( "p3" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p3 = Vector3(x, y, z);

		manager_.AddGround(p0, p1, p2, p3);
	}
	if(name == "verticalchess")
	{
		unsigned int depth;
		xis >> xml::attribute( "depth", depth )
		>>xml::start( "p0" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p0 = Vector3(x, y, z);
		xis >> xml::start( "p1" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p1 = Vector3(x, y, z);
		manager_.GenerateVerticalChess(p0, p1, depth);
	}
	if(name == "prise")
	{
		xis >>xml::start( "p0" )
				>> xml::attribute( "x", x )
				>> xml::attribute( "y", y )
				>> xml::attribute( "z", z )
		>> xml::end;
		p0 = Vector3(x, y, z);
		Vector3 pu(p0(0), p0(1)+0.1, p0(2)+0.1);
		Vector3 pb(p0(0), p0(1)-0.1, p0(2)-0.1);
		manager_.GenerateVerticalChess(pu, pb, 0);
	}
	if(name == "color")
	{
		float r, g, b, t;
		xis >>xml::start( "p0" )
				>> xml::attribute( "r", r )
				>> xml::attribute( "g", g )
				>> xml::attribute( "b", b )
				>> xml::attribute( "t", t )
		>> xml::end;
		manager_.SetNextColor(r, g, b);
		manager_.SetNextTransparency(t);

	}
	if(name == "texture")
	{
		int tex = 0;
		xis >>xml::start( "p0" )
				>> xml::attribute( "index", tex )
		>> xml::end;
		manager_.SetNextTexture(tex);

	}
}
