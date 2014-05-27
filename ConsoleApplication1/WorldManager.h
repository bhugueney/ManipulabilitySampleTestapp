#include "world\World.h"
#include "tools\MatrixDefs.h"
#include <vector>

using namespace matrices;
using namespace manip_core;
typedef std::vector<Obstacle*> Quads;
namespace Manager{
	class WorldManager{
	public:
		WorldManager();
		~WorldManager();
	public:
		World * initWorld();
		Quads& getQuads();

	private:
		World* buildWorld();

		void setNextColor(float /*r*/, float /*g*/, float /*b*/);
		void addObstacle(const Vector3 /*upleft*/, const Vector3 /*upright*/, const Vector3 /*downright*/, const Vector3 /*downleft*/, World &world);
		void generateStairChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, double heightInit, double heightFinal, const unsigned int depth, const unsigned int chessdepth, World&);
		void generateChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, double height, const unsigned int depth, World&);
		void generateXInclinedPlank(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, World&);
		void initialize(bool, World&);

	private:
		float color_[3];

		Quads quads;
	};
}