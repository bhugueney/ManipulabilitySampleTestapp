
#ifndef _CLASS_IKCONSTRAINTHANDLER
#define _CLASS_IKCONSTRAINTHANDLER

#include "API/IkConstraintHandlerI.h"
#include "API/IkConstraintHandlerI.h"
#include "IKSolver.h"

#include "world/World.h"

#include "MatrixDefs.h"
#include <memory>
#include <map>

class PartialDerivativeConstraint;


struct ConstraintHandlerPimpl;
class IkConstraintHandler : public manip_core::IkConstraintHandlerI{

public:
	 IkConstraintHandler(const World& /*World*/);
	~IkConstraintHandler();
	
/*inherited*/
public:
	virtual bool AddConstraint   (const manip_core::enums::IKConstraints::eIKConstraints /*constraint*/);
	virtual bool RemoveConstraint(const manip_core::enums::IKConstraints::eIKConstraints /*constraint*/);
	virtual void Release();
	
public:
	typedef std::map<int, PartialDerivativeConstraint*> T_Constraint;	
	typedef T_Constraint::iterator T_ConstraintIT;
	typedef T_Constraint::const_iterator T_ConstraintCIT;

public:
	const T_Constraint& GetConstraints() const;

private:
	std::auto_ptr<ConstraintHandlerPimpl> pImpl_;

};

#endif //_CLASS_IKCONSTRAINTHANDLER