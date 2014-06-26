#include "WorldParserObj.h"

#include "Simulation.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

using namespace matrices;
using namespace std;

WorldParserObj::WorldParserObj()
	:manager_(Simulation::GetInstance()->manager_)
{
	// NOTHING
}

WorldParserObj::~WorldParserObj()
{
	// NOTHING
}

namespace
{
string doubleSlash(const string& s)
{
    //Remplace "//" par "/1/".
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(i<s.size()-1&&s[i]=='/'&&s[i+1]=='/')
        {
            s1+="/1/";
            i++;
        }
        else
            s1+=s[i];
    }
    return s1;
}

string remplacerSlash(const string& s)
{
    //Remplace les '/' par des espaces.
    string ret="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]=='/')
            ret+=' ';
        else
            ret+=s[i];
    }
    return ret;
}

vector<string> splitSpace(const string& s)
{
    //Eclate une chaîne au niveau de ses espaces.
    vector<string> ret;
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]==' '||i==s.size()-1)
        {
            if(i==s.size()-1)
                s1+=s[i];
            ret.push_back(s1);
            s1="";
        }
        else
            s1+=s[i];
    }
    return ret;
}
}

void WorldParserObj::CreateWorld(const std::string& filename, const bool isGround)
{
	string line;
    ifstream myfile (filename.c_str());
	std::vector<std::string> lines;
	if (myfile.is_open())
	{
		while ( myfile.good() )
		{
			getline (myfile, line);
			if(line.find("t ") == 0)
			{
				char t[255];
				sscanf(line.c_str(),"t %s",t);
				manager_.SetNextTexture(strtod (t, NULL));
			}
			if(line.find("c ") == 0)
			{
				char r[255],g[255],b[255],t[255];
				sscanf(line.c_str(),"c %s %s %s %s",r,g,b,t);
				manager_.SetNextColor(strtod (r, NULL), strtod(g, NULL), strtod(b, NULL));
				manager_.SetNextTransparency(strtod (t, NULL));
			}
			if(line.find("v ") == 0)
			{
				char x[255],y[255],z[255];
				sscanf(line.c_str(),"v %s %s %s",x,z,y);
				points_.push_back(Vector3(-strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
			}
			if(line.find("vn ") == 0)
			{
				char x[255],y[255],z[255];
				sscanf(line.c_str(),"vn %s %s %s",x,z,y);
				normals_.push_back(Vector3(-strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
			}
			if(line.find("f ") == 0)
			{
				lines.push_back(line);
				if(lines.size() == 2)
				{
					CreateObstacle(lines, isGround);
					lines.clear();
				}
			}
		}
		myfile.close();
	}
}


namespace
{
    typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices::Vector3> > T_Point;

	NUMBER isLeft( const VectorX& P0, const VectorX& P1, const Vector3& P2 )
	{
		return ( (P1.x() - P0.x()) * (P2.y() - P0.y())
				- (P2.x() - P0.x()) * (P1.y() - P0.y()) );
	}

	const Vector3& LeftMost(const T_Point& points)
	{
		unsigned int i = 0;
		for(unsigned int j = 1; j < points.size(); ++j)
		{
			if(points[j].x() < points[i].x())
			{
				i = j;
			}
		}
		return points[i];
	}

	T_Point ConvexHull(const T_Point& points)
	{
		T_Point res;
		Vector3 pointOnHull = LeftMost(points);	
		Vector3 endPoint;
		int i = 0;
		do
		{
			++i;
			VectorX pi = pointOnHull;
			endPoint = points[0];
			for(unsigned int j = 1; j < points.size(); ++j)
			{
				if((endPoint == pointOnHull) || (isLeft(pi, endPoint, points[j]) > 0))
				{
					endPoint = points[j];
				}
				
				if( i > 10000 )
				{
					std::cout << " WTF " << std::endl << points[j] << std::endl;
					bool gtd = false;
				}
			}
			res.push_back(pi);
			pointOnHull = endPoint;
		} while(endPoint != res[0]);
		res.push_back(endPoint);
		return res;
	}
}

using namespace matrices;

void WorldParserObj::CreateObstacle(const std::vector<std::string>& lines, bool isGround)
{
	vector<long int> indices;
    T_Point points;
	for(int i =0; i<2; ++i)
	{
		string ligne = doubleSlash(lines[i]);
		ligne=remplacerSlash(ligne); //On remplace les '/' par des espaces, ex : pour "f 1/2/3 4/5/6 7/8/9" on obtiendra "f 1 2 3 4 5 6 7 8 9"
		vector<string> termes=splitSpace(ligne.substr(2)); //On éclate la chaîne en ses espaces (le substr permet d'enlever "f ")
		int ndonnees=(int)termes.size()/3;
		for(int i=0; i <ndonnees;i++) //On aurait très bien pu mettre i<ndonnees mais je veux vraiment limiter à 3 ou 4
		{
			long int idx = (strtol(termes[i*3].c_str(),NULL, 10)) - 1;
			std::vector<long int>::iterator it = indices.begin();
			it = find (indices.begin(), indices.end(), idx);
			if(it == indices.end())
			{
				indices.push_back(idx);
				points.push_back(points_[(int)idx]);
			}
		}
	}
     
    //T_Point points;
	//for(int i =0; i<4; ++i)
	//{
	//	char x[255],y[255],z[255];
	//	sscanf(lines[i].c_str(),"v %s %s %s",x,z,y);
	//	points.push_back(Vector3(-strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
	//}

	Vector3 p1(points[0]);
	Vector3 p2(points[1]);
	Vector3 p3(points[2]);
	Vector3 p4(points[3]);
	Vector3 u_(p3-p4);
	Vector3 v_(p1-p4);
	if(abs( u_.dot(v_)) > 0.001) v_ = p2 - p4;
	double a_, b_, c_, d_, norm_, normsquare_;
	//we need convex hull of this crap
	Vector3 normal (u_.cross(v_));
	a_ = (float)(normal.x());
	b_ = (float)(normal.y());
	c_ = (float)(normal.z());
	//if (c_ < 0) c_ = -c_;
	norm_ = (float)(normal.norm());
	normsquare_ = norm_ * norm_;
	d_ = (float)(-(a_ * p1.x() + b_ * p1.y() + c_ * p1.z()));

	Matrix4 basis_ = Matrix4::Zero();
	Vector3 x = u_; x.normalize();
	Vector3 y = v_; y.normalize();
	normal.normalize();
	basis_.block(0,0,3,1) = x;
	basis_.block(0,1,3,1) = y;
	basis_.block(0,2,3,1) = normal;
	basis_.block(0,3,3,1) = p4;
	basis_(3,3) = 1;
	Matrix4 basisInverse_ = basis_.inverse();

	T_Point transformedPoints;
	for(int i=0; i<4; ++i)
	{
		transformedPoints.push_back(matrices::matrix4TimesVect3(basisInverse_, points[i]));
	}

	points = ConvexHull(transformedPoints);
	transformedPoints.clear();
	for(int i=0; i<4; ++i)
	{
		transformedPoints.push_back(matrices::matrix4TimesVect3(basis_, points[i]));
	}

	// make sure normal in the good sense :
	u_ = transformedPoints[2] - transformedPoints[3];
	v_ = transformedPoints[0] - transformedPoints[3];

	normal = u_.cross(v_);
	normals_[(int)indices[0]];
	if(isGround)
	{
		if(normal.dot(normals_[(int)indices[0]]) > 0 )
		{
			manager_.AddGround(transformedPoints[0], transformedPoints[1], transformedPoints[2], transformedPoints[3]);
		}
		else
		{
			manager_.AddGround(transformedPoints[2], transformedPoints[1], transformedPoints[0], transformedPoints[3]);
		}
	}
	else
	{
		if(normal.dot(normals_[(int)indices[0]]) > 0 )
		{
			manager_.AddObstacle(transformedPoints[0], transformedPoints[1], transformedPoints[2], transformedPoints[3]);
		}
		else
		{
			manager_.AddObstacle(transformedPoints[2], transformedPoints[1], transformedPoints[0], transformedPoints[3]);
		}
	}
}
