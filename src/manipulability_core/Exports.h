#ifdef _WIN32
  #ifdef MANIPCORE_DLLEXPORT
    #define MANIPCORE_API __declspec(dllexport)
  #else
    #define MANIPCORE_API __declspec(dllimport)
  #endif
#else
  #define  MANIPCORE_API
#endif
