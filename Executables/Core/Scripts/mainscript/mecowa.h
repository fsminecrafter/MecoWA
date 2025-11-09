#pragma once

#include <string>

int windowWidth = 640;
int windowHeight = 480;

std::string version = "0.03"; //MecoWA version

// Global engine settings
inline float cameraSpeed = 3.0f;           // base movement speed
inline float cameraSensitivity = 0.1f;     // mouse sensitivity
inline float cameraPitchClamp = 89.0f;     // prevents flipping
inline bool invertMouseY = true;          // invert up/down
inline bool invertMouseX = false;          // invert left/right