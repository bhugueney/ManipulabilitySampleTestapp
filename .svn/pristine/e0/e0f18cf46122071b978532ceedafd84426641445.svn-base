
#ifndef _CLASS_TEXTUREHANDLER
#define _CLASS_TEXTUREHANDLER

#include <memory>
#include <string>

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>


namespace textures
{
	struct PImpl;

	enum eTextureType
	{ 
		wood = 0,
		wall,
		rock,
		Unknown
	};

class TextureHandler {

public:
	  TextureHandler(const std::string& filepath);
	 ~TextureHandler();

public:
	void EnableTexture(eTextureType /*etype*/);
	void DisableTextures();


private:
	std::auto_ptr<PImpl> pImpl_;
}; // TextureHandler

}

#endif //_CLASS_TEXTUREHANDLER