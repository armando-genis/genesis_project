#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <vector>
#include <iostream>

class Node {
public:
    float gCost;
    float hCost;

    struct Vector3 {
        float x, y, z;
        Vector3() : x(0), y(0), z(0) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
    };

    Vector3 rearWheelPos;
    Vector3 frontWheelPos;
    Vector3 reverseWheelPos;
    float heading;
    bool isReversing;
    float trailerHeading;

    Node* previousNode;

    int heapIndex;

    Node() : gCost(0), hCost(0), heading(0), isReversing(false), trailerHeading(0), previousNode(nullptr), heapIndex(0) {}

    Node(Node* previousNode, Vector3 rearWheelPos, float heading, bool isReversing)
        : previousNode(previousNode), rearWheelPos(rearWheelPos), heading(heading), isReversing(isReversing), gCost(0), hCost(0), trailerHeading(0), heapIndex(0) {}

    void AddCosts(float gCost, float hCost) {
        this->gCost = gCost;
        this->hCost = hCost;
    }

    float fCost() const {
        return gCost + hCost;
    }

    float HeadingInRadians() const {
        return heading;
    }

    float HeadingInDegrees() const {
        return heading * 180.0 / M_PI;
    }

    float TrailerHeadingInRadians() const {
        return trailerHeading;
    }

    void SetTrailerHeadingInRadians(float heading) {
        trailerHeading = heading;
    }

    float TrailerHeadingInDegrees() const {
        return trailerHeading * 180.0 / M_PI;
    }

    void StealDataFromThisNode(Node* other) {
        other->gCost = gCost;
        other->hCost = hCost;
        other->rearWheelPos = rearWheelPos;
        other->frontWheelPos = frontWheelPos;
        other->heading = heading;
        other->isReversing = isReversing;
        other->previousNode = previousNode;
        other->trailerHeading = trailerHeading;
    }

    int GetHeapIndex() const {
        return heapIndex;
    }

    void SetHeapIndex(int index) {
        heapIndex = index;
    }

    int CompareTo(const Node* nodeToCompare) const {
        if (fCost() == nodeToCompare->fCost()) {
            return hCost < nodeToCompare->hCost ? -1 : hCost > nodeToCompare->hCost ? 1 : 0;
        }
        return fCost() < nodeToCompare->fCost() ? -1 : 1;
    }
};

#endif // NODE_H
