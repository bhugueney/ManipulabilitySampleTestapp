
#include "MatrixDefs.h"
#include <gl/gl.h>
#include <gl/glu.h>


namespace input
{
	static float mousePos[3] = {0.f, 0.f, 0.f};

	void GetMouseClicked(const matrices::Vector2 pos, float* mousePos)
	{
		GLint viewport[4];
		GLdouble modelview[16];
		GLdouble projection[16];
		GLfloat winX, winY, winZ;
		GLdouble posX, posY, posZ;
 
		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetIntegerv( GL_VIEWPORT, viewport );
 

		POINT mouse;                        // Stores The X And Y Coords For The Current Mouse Position
		GetCursorPos(&mouse);                   // Gets The Current Cursor Coordinates (Mouse Coordinates)
		int success = ScreenToClient(WindowFromDC(wglGetCurrentDC()), &mouse);
	
		winX = (float)mouse.x;                  // Holds The Mouse X Coordinate
		winY = (float)mouse.y; 
		winY = (float)viewport[3] - winY;

		/*winX = (float)pos.x();
		winY = (float)viewport[3] - (float)pos.y();*/
		glReadPixels( (int)winX, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
 
		gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
 
		mousePos[0] = (float)posX;
		mousePos[1] = (float)posY;
		mousePos[2] = (float)posZ;
	}
}