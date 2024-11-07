#ifndef STATE_INDEX_MANAGER_H
#define STATE_INDEX_MANAGER_H

class HDStateIndexManager {
public:
    static HDStateIndexManager& getInstance() {
        static HDStateIndexManager instance;
        return instance;
    }

    int HDgetNextIndex() {
        return stateIndex++;
    }

private:
    HDStateIndexManager() : stateIndex(0) {}
    int stateIndex;
};

#endif // STATE_INDEX_MANAGER_H