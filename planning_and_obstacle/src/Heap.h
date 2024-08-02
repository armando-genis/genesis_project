#ifndef HEAP_H
#define HEAP_H

#include <vector>
#include <stdexcept>
#include <algorithm>

template <typename T>
class Heap {
public:
    explicit Heap(int maxHeapSize) {
        items.reserve(maxHeapSize);
    }

    void Add(T* item) {
        if (currentItemCount >= items.capacity()) {
            throw std::runtime_error("Heap is full");
        }

        item->SetHeapIndex(currentItemCount);
        items.push_back(item);
        SortUp(item);
        currentItemCount++;
    }

    T* RemoveFirst() {
        if (currentItemCount == 0) {
            throw std::runtime_error("Heap is empty");
        }

        T* firstItem = items[0];
        currentItemCount--;

        items[0] = items[currentItemCount];
        items[0]->SetHeapIndex(0);
        items.pop_back();
        SortDown(items[0]);

        return firstItem;
    }

    int Count() const {
        return currentItemCount;
    }

    bool Contains(T* item) const {
        return items[item->GetHeapIndex()] == item;
    }

    void UpdateItem(T* item) {
        SortUp(item);
    }

    void Clear() {
        items.clear();
        currentItemCount = 0;
    }

private:
    std::vector<T*> items;
    int currentItemCount = 0;

    void SortDown(T* item) {
        while (true) {
            int childIndexLeft = item->GetHeapIndex() * 2 + 1;
            int childIndexRight = item->GetHeapIndex() * 2 + 2;
            int swapIndex = 0;

            if (childIndexLeft < currentItemCount) {
                swapIndex = childIndexLeft;

                if (childIndexRight < currentItemCount) {
                    if (items[childIndexLeft]->CompareTo(items[childIndexRight]) < 0) {
                        swapIndex = childIndexRight;
                    }
                }

                if (item->CompareTo(items[swapIndex]) < 0) {
                    Swap(item, items[swapIndex]);
                } else {
                    return;
                }
            } else {
                return;
            }
        }
    }

    void SortUp(T* item) {
        int parentIndex = (item->GetHeapIndex() - 1) / 2;

        while (true) {
            T* parentItem = items[parentIndex];

            if (item->CompareTo(parentItem) > 0) {
                Swap(item, parentItem);
            } else {
                break;
            }

            parentIndex = (item->GetHeapIndex() - 1) / 2;
        }
    }

    void Swap(T* itemA, T* itemB) {
        items[itemA->GetHeapIndex()] = itemB;
        items[itemB->GetHeapIndex()] = itemA;

        int itemAIndex = itemA->GetHeapIndex();
        itemA->SetHeapIndex(itemB->GetHeapIndex());
        itemB->SetHeapIndex(itemAIndex);
    }
};

#endif // HEAP_H
