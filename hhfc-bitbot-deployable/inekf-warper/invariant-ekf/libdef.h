#pragma once

#ifdef _OS_WINDOWS_
#ifdef export_inekf
#define INEKF_API __declspec(dllexport)
#else
#define INEKF_API __declspec(dllimport)
#endif // export_inekf

#else
#define INEKF_API
#endif // _OS_WINDOWS_