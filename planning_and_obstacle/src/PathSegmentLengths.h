#ifndef PATHSEGMENTLENGTHS_H
#define PATHSEGMENTLENGTHS_H

#include <Eigen/Dense>

class PathSegmentLengths {
public:
    float t;
    float u;
    float v;

    PathSegmentLengths(float t = 0, float u = 0, float v = 0)
        : t(t), u(u), v(v) {}
};

#endif // PATHSEGMENTLENGTHS_H
