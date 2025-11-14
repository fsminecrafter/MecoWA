#pragma once

#include <string>

extern int windowWidth; //640
extern int windowHeight; //480
extern float airDensity; //1.225 kg/m^3
extern float gravityG; //1.0f

extern std::string version;

// Global engine settings
inline float cameraSpeed = 3.0f;           // base movement speed
inline float cameraSensitivity = 0.1f;     // mouse sensitivity
inline float cameraPitchClamp = 89.0f;     // prevents flipping
inline bool invertMouseY = true;          // invert up/down
inline bool invertMouseX = false;          // invert left/right