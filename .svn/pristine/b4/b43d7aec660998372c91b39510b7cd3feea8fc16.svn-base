
#ifndef _CLASS_COM_VISITOR_ABC
#define _CLASS_COM_VISITOR_ABC

#include "MatrixDefs.h"
#include "Exports.h"

class ComVisitor_ABC {
public:
	 ComVisitor_ABC();
	 ~ComVisitor_ABC();

	virtual void VisitCom(const matrices::Vector3& /*position*/, const float& /*weight*/) = 0;

private:
	 ComVisitor_ABC& ComVisitor_ABC::operator =(const ComVisitor_ABC&);
	 ComVisitor_ABC(const ComVisitor_ABC&);
}; // ComVisitor_ABC

#endif //_CLASS_COM_VISITOR_ABC