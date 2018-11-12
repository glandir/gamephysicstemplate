#pragma once

#include "Point.h"

void eulerIntegrate(Points& points, float dt);
void midpointIntegrate1(Points& points, OldPoints& oldpoints, float dt);
void midpointIntegrate2(Points& points, OldPoints& oldpoints, float dt);
void leapfrogIntegrate(Points& points, float dt);