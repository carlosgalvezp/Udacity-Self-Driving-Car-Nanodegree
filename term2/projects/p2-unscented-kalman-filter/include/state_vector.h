#ifndef STATE_VECTOR_H
#define STATE_VECTOR_H

#include <Eigen/Dense>
#include "tools.h"

static const int n_states = 5;

typedef Eigen::Matrix<double, n_states, 1> StateVectorBase;

class StateVector : public StateVectorBase
{
public:
    enum Indices
    {
        indexPosX     = 0,
        indexPosY     = 1,
        indexVelocity = 2,
        indexYaw      = 3,
        indexYawRate  = 4
    };

    StateVector()
    {
    }

    template<typename T>
    StateVector(const T& x)
        : StateVectorBase(x)
    {
    }

    template<typename T>
    StateVector& operator=(const T& x)
    {
        StateVectorBase::operator=(x);
        return *this;
    }

    template<typename T>
    StateVector operator-(const T& x)
    {
        StateVector y = StateVectorBase::operator-(x);
        y[indexYaw] = Tools::normalizeAngle(y[indexYaw]);

        return y;
    }
};

#endif // STATE_VECTOR_H
