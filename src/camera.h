#ifndef CAMERA_H
#define CAMERA_H

#include <cmath>

class Camera
{
  double x;        // the current x position
  double y;        // the current y position
  double z;        // the current z position
  double target_x; // the current x position where camera is looking
  double target_y; // the current y position where camera is looking
  double target_z; // the current z position where camera is looking
  double up_x;     // the current x position of the up axis
  double up_y;     // the current y position of the up axis
  double up_z;     // the current z position of the up axis
  double dp;       // increment for translating the camera up/down
  double angleX;    // angle of y-rotation for the camera direction
  double angleY;    // angle of x-rotation for the camera direction
public:
  Camera() : x(1), y(3), z(-1), dp(0.2), angleX(0.0), angleY(0.0)
  {
    target_x = x - sin(angleX);
    target_y = y + sin(angleY);
    target_z = z + cos(angleX);
    up_x = 0;
    up_y = 1;
    up_z = 0;
  }
  double getX() { return x; }
  double getY() { return y; }
  double getZ() { return z; }
  double getTargetX() { return target_x; }
  double getTargetY() { return target_y; }
  double getTargetZ() { return target_z; }
  double getUpX() { return up_x; }
  double getUpY() { return up_y; }
  double getUpZ() { return up_z; }
  double getAngleX() {return angleX;}
  double getAngleY() {return angleY;}

  void setX(double _x) { x = _x; target_x = x - sin(angleX);}
  void setY(double _y) { y = _y; target_y = y + sin(angleY);}
  void setZ(double _z) { z = _z; target_z = z + cos(angleX);}
  void moveRight() { x -= dp; target_x = x - sin(angleX);}
  void moveLeft() { x += dp; target_x = x - sin(angleX);}
  void moveUp() { y += dp; target_y = y + sin(angleY);}
  void moveDown() { y -= dp; target_y = y + sin(angleY);}
  void moveForward() { z += dp; target_z = z + cos(angleX);}
  void moveBack() { z -= dp; target_z = z + cos(angleX);}
  void setAngleX(double _angleX) { angleX = _angleX; setX(getX()); setZ(getZ());}
  void setAngleY(double _angleY) { angleY = _angleY; setY(getY()); setZ(getZ());}
};

#endif