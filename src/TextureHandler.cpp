#include "TextureHandler.h"

using namespace textures;

class Image {
  int image_width,image_height;
  byte *image_data;
public:
  Image (char *filename);
  // load from PPM file
  ~Image();
  int width() { return image_width; }
  int height() { return image_height; }
  byte *data() { return image_data; }
};


// skip over whitespace and comments in a stream.

static void skipWhiteSpace (char *filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
//    if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);

    // skip comments
    if (c == '#') {
      do {
	d = fgetc(f);
	//if (d==EOF) dsError ("unexpected end of file in \"%s\"",filename);
      } while (d != '\n');
      continue;
    }

    if (c > ' ') {
      ungetc (c,f);
      return;
    }
  }
}


// read a number from a stream, this return 0 if there is none (that's okay
// because 0 is a bad value for all PPM numbers anyway).

static int readNumber (char *filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    //if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}

#include <iostream>

Image::Image (char *filename)
{
  FILE *f = fopen (filename,"rb");
  if (!f) 
	{
	std::cout << "Can't open image file " << filename ;
  }

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '6')
    std::cout << "image file" << filename << " is not a binary PPM (no P6 header)";
  skipWhiteSpace (filename,f);

  // read in image parameters
  image_width = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  image_height = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  int max_value = readNumber (filename,f);

  // check values
  /*if (image_width < 1 || image_height < 1)
    dsError ("bad image file \"%s\"",filename);
  if (max_value != 255)
    dsError ("image file \"%s\" must have color range of 255",filename);*/

  // read either nothing, LF (10), or CR,LF (13,10)
  int c = fgetc(f);
  if (c == 10) {
    // LF
  }
  else if (c == 13) {
    // CR
    c = fgetc(f);
    if (c != 10) ungetc (c,f);
  }
  else ungetc (c,f);

  // read in rest of data
  image_data = new byte [image_width*image_height*3];
 /* if (fread (image_data,image_width*image_height*3,1,f) != 1)
    dsError ("Can not read data from image file `%s'",filename);*/
  fclose (f);
}


Image::~Image()
{
  delete[] image_data;
}

//***************************************************************************
// Texture object.

class Texture {
  Image *image;
  GLuint name;
public:
  Texture (char *filename);
  ~Texture();
  void bind (int modulate);
};


Texture::Texture (char *filename)
{
  image = new Image (filename);
  glGenTextures (1,&name);
  glBindTexture (GL_TEXTURE_2D,name);

  // set pixel unpacking mode
  glPixelStorei (GL_UNPACK_SWAP_BYTES, 0);
  glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei (GL_UNPACK_SKIP_ROWS, 0);
  glPixelStorei (GL_UNPACK_SKIP_PIXELS, 0);

  // glTexImage2D (GL_TEXTURE_2D, 0, 3, image->width(), image->height(), 0,
  //		   GL_RGB, GL_UNSIGNED_BYTE, image->data());
  gluBuild2DMipmaps (GL_TEXTURE_2D, 3, image->width(), image->height(),
		     GL_RGB, GL_UNSIGNED_BYTE, image->data());

  // set texture parameters - will these also be bound to the texture???
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		   GL_LINEAR_MIPMAP_LINEAR);

  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}


Texture::~Texture()
{
  delete image;
  glDeleteTextures (1,&name);
}


void Texture::bind (int modulate)
{
  glBindTexture (GL_TEXTURE_2D,name);
  glTexEnvi (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,
	     modulate ? GL_MODULATE : GL_DECAL);
}

#include <map>

namespace textures
{
struct PImpl
{
	PImpl(const std::string& filepath)
		: filepath_(filepath)
	{
		// NOTHING
	}

	~PImpl()
	{
		for(std::map<textures::eTextureType, Texture*>::iterator it = textures_.begin(); it != textures_.end(); ++it)
		{
			delete(it->second);
		}
	}
	std::map<textures::eTextureType, Texture*> textures_;
	const std::string filepath_;
};
}

using namespace textures;


TextureHandler::TextureHandler(const std::string& filepath)
	: pImpl_(new PImpl(filepath))
{
	// load all textures
}

TextureHandler::~TextureHandler()
{
	// NOTHING
}

void TextureHandler::EnableTexture(eTextureType etype)
{
	std::map<textures::eTextureType, Texture*>::iterator it = pImpl_->textures_.find(etype);
	if( it == pImpl_->textures_.end())
	{
		std::string file;
		switch(etype)
		{
			case textures::rock:
			{
				file = pImpl_->filepath_ + "/rock.ppm";
				break;
			}
			case textures::wall:
			{
				file = pImpl_->filepath_ + "/wall.ppm";
				break;
			}
			case textures::wood:
			{
				file = pImpl_->filepath_ + "/wood.ppm";
				break;
			}
			default:
				return;
		}
		const char *prefix = file.c_str();
		char *s = (char*) alloca (strlen(prefix) + 20);

		strcpy (s,prefix);
		pImpl_->textures_.insert(std::make_pair(etype, new Texture(s)));
	}
	glEnable (GL_TEXTURE_2D);
	pImpl_->textures_[etype]->bind(1);
      glEnable (GL_TEXTURE_GEN_S);
      glEnable (GL_TEXTURE_GEN_T);
      glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      static GLfloat s_params[4] = {1.0f,1.0f,0.0f,1};
      static GLfloat t_params[4] = {0.817f,-0.817f,0.817f,1};
      glTexGenfv (GL_S,GL_OBJECT_PLANE,s_params);
      glTexGenfv (GL_T,GL_OBJECT_PLANE,t_params);
	//glDisable (GL_LIGHTING);
	//glShadeModel (GL_FLAT);
	//glEnable (GL_DEPTH_TEST);
	//glDepthFunc (GL_LESS);
	//  // glDepthRange (1,1);
	//glEnable (GL_TEXTURE_2D);
	//pImpl_->textures_[etype]->bind(1);
}

void TextureHandler::DisableTextures()
{
      glDisable (GL_TEXTURE_2D);
}