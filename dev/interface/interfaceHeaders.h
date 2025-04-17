// interfaceHeaders.hpp
#pragma once
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdio.h>
#include <iostream>
#include "inference_net.hpp"
#include "Joystick.h"

#define _JNT_NUM 20

#define _D_INTERFACE_BEGIN namespace Dcc { namespace INTERFACE_BITBOT {
#define _D_INTERFACE_END }}
#define _D_INTERFACE ::Dcc::INTERFACE_BITBOT::
#define _USING_D_INTERFACE using namespace Dcc::INTERFACE_BITBOT;