#ifdef XBOXCONTROLLER_DLLEXPORT
	#define XBOXCONTROLLER_API __declspec(dllexport)
#else
	#define XBOXCONTROLLER_API __declspec(dllimport)
#endif

