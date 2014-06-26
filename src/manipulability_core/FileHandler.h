#ifndef _CLASS_FILEHANDLER
#define _CLASS_FILEHANDLER

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>

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
		std::ofstream myfile;
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

	FileHandler& operator<< (const double& val)
	{
		bvhStr_ << val;
		return *this;
	}
	
	FileHandler& operator<< (const float& val)
	{
		bvhStr_ << val;
		return *this;
	}
	
	FileHandler& operator<< (const int& val)
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

#endif //_CLASS_TIMER