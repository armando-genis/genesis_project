#ifndef IHEAPITEM_H
#define IHEAPITEM_H

template <typename T>
class IHeapItem {
public:
    virtual int GetHeapIndex() const = 0;
    virtual void SetHeapIndex(int index) = 0;
    virtual int CompareTo(const T* other) const = 0;
};

#endif // IHEAPITEM_H
