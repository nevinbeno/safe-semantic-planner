#pragma once
#include "PlanningProblem.hpp"
#include "PlanningResult.hpp"

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};
