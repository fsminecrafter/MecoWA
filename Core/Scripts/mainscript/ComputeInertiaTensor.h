#pragma once

#include "engine.h"

glm::dmat3 ComputeInertiaTensor(const ModelInstance& inst, const glm::dvec3& cg, double density);