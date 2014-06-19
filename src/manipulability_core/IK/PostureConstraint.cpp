
#include "PostureConstraint.h"

#include "kinematic/Robot.h"
#include "kinematic/Tree.h"

#include "kinematic/Jacobian.h"
#include "sampling/Sample.h"

using namespace matrices;
using namespace Eigen;

PostureConstraint::PostureConstraint()
{
	// NOTHING
}

PostureConstraint::~PostureConstraint()
{
	// NOTHING
}

#include "Pi.h"

NUMBER PostureConstraint::Evaluate(const Robot& robot, const Tree& tree, const int joint, Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3& direction)
{
	Sample * target = tree.targetSample_;
	if( joint == 3 )///*tree.GetTreeType() == manip_core::enums::RightArmEscalade &&*/ (joint == 1) )
	{
		if(target)
		{
			const Joint* j = tree.GetRoot(); int i =1;
			while (j)
			{
				if(i  == joint)
				{
					double angleRef = target->AngleValues()[i];
					while (angleRef > 360 * DegreesToRadians )
					{
						angleRef -= 360 * DegreesToRadians;
					}
					while (angleRef < -360 * DegreesToRadians )
					{
						angleRef += 360 * DegreesToRadians;
					}
					if(angleRef < 0)
					{
						angleRef = (360 * DegreesToRadians + angleRef);
					}
					double angle = j->GetAngle();
					double angleRefOpposed = angleRef > 180  * DegreesToRadians ? (angleRef - 180  * DegreesToRadians ) : (angleRef + 180  * DegreesToRadians);
					if(angleRefOpposed == 360 * DegreesToRadians)
					{
						angleRefOpposed = 0;
					}
					int sign = 1;
					if(angleRefOpposed < angleRef && angle < angleRefOpposed)
					{
						sign = -1;
					}
					else if(angleRefOpposed > angleRef && angle >  angleRef)
					{
						sign = -1;
					}
					if(abs(angleRefOpposed - angle) <  10)
					{
						sign = -1;
					}
					double res =  sign * abs((angleRef-angle)) / (360 * DegreesToRadians);
					if( abs(angleRef-angle) < 5 * DegreesToRadians )
					{
						return 0;
					}
					else
					{
						res = res > 1 ? 1 : res;
						res = res < -1 ? -1 : res;
						return res;
					}
					/*std::cout << "joint " << i << " target   :" << angleRef << std::endl;
					std::cout << "joint " << i << " current   :" << angle << std::endl;
					std::cout << "joint " << i << " distance   :" << ((angleRef - (angle + epsilon)) * (angleRef - (angle + epsilon)) - (angleRef - (angle - epsilon)) * (angleRef - (angle - epsilon))) /(epsilon * 2) << std::endl;
					*///return ((angleRef - (angle + epsilon)) * (angleRef - (angle + epsilon)) - (angleRef - (angle - epsilon)) * (angleRef - (angle - epsilon))) /(epsilon * 2);
				}
				else
				{
					j = j->pChild_;
				}
				++i;
			}
		}
	}
	return 0;
}



