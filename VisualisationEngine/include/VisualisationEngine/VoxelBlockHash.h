#ifndef VOXELBLOCKHASH_H_
#define VOXELBLOCKHASH_H_

#include <Eigen/Eigen>
#include <climits>
#include <cmath>
#include <cstdint>
#include <optional>

// 一个体素块边长/一个体素边长
constexpr int SDF_BLOCK_SIZE = 8;

// 一个体素块所容纳的体素个数
constexpr int SDF_BLOCK_SIZE3 = 512;

// 哈希表掩码
constexpr uint32_t SDF_HASH_MASK = 0xfffff;

constexpr uint32_t SDF_BUCKET_NUM = 0x100000;

constexpr uint32_t SDF_EXCESS_LIST_SIZE = 0x20000;

// 内存管理(固定内存池)
template <typename T>
class MemoryMannager {
public:
    MemoryMannager(int max_capacity) {
        ptr_ = new T[max_capacity];
        free_blocks_.reserve(max_capacity);
        for (int i{0}; i < max_capacity; ++i) free_blocks_[i] = i;
    }

    std::optional<T*> allocate() {
        if (free_blocks_.empty())
            return std::nullopt;
        else
            return ptr_[free_blocks_.pop_back()];
    }

    void free(T* ptr) { free_blocks_.push_back(ptr - ptr_); }

    T& operator()(int id) { return ptr_[id]; }

    T* get_data() { return ptr_; }

private:
    // 指向申请的堆内存
    T* ptr_{nullptr};

    // 存储空闲指针的栈
    std::vector<int> free_blocks_;
};

// 哈希表实体
struct HashEntry {
    // 指向的体素块的坐标
    Eigen::Vector3i pos{Eigen::Vector3i::Zero()};
    // 发生冲突时，指向的冲突列表元素的全局偏移量
    int offset{-1};
    // 指向的体素块地址偏移量
    int ptr;
    // 该所对应的体素块是否被使用标志
    bool isUsed{false};
};

// 体素
struct Voxel {
    short sdf;
    u_char w_depth;
};

// 体素哈希表
class VoxelBlockHash {
public:
    static const int noTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;

    inline static int getHashIndex(Eigen::Vector3i blockPos) {
        return (((uint)blockPos(0) * 73856093u) ^ ((uint)blockPos(1) * 19349669u) ^
                ((uint)blockPos(2) * 83492791u)) &
               (uint)SDF_HASH_MASK;
    }

    VoxelBlockHash()
        : hashEntries_(noTotalEntries), localVoxels_(noTotalEntries * SDF_BLOCK_SIZE3) {}

    HashEntry* get_entryData() { return hashEntries_.get_data(); }

    Voxel* get_voxelData() { return localVoxels_.get_data(); }

    // 截断距离
    float mu_;

    // 一个体素的边长，其实就是分辨率
    float voxelSize_;

    // tsdf中的最大权重
    float max_w_;

private:
    MemoryMannager<HashEntry> hashEntries_;
    MemoryMannager<Voxel> localVoxels_;
};

struct SceneParams {
    // 截断距离
    float mu;

    // 最大权重
    float maxWeight;

    // 一个体素的边长，其实就是建图的分辨率
    float voxelSize;
};

// TODO：当前版本不支持动态内存
class Scene {
public:
    Scene(const SceneParams& sceneParams) : sceneParams_(sceneParams) {
        hashEntries_ = new HashEntry[SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE];
        voxel_ = new Voxel[(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * SDF_BLOCK_SIZE3];
#pragma omp parallel for
        for (int i = 0; i < (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE); ++i) hashEntries_[i].ptr = i;
        freeExcessEntries_.resize(SDF_EXCESS_LIST_SIZE);
        for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i)
            freeExcessEntries_[i] = SDF_BUCKET_NUM + i + 1;
    }

    ~Scene() {
        delete[] voxel_;
        delete[] hashEntries_;
    }

    // 预分配可视体素块列表内存
    void reserveVisibleVoxelBlockList(int size) {
        lastFrametVisibleVoxelBlockList_.reserve(size);
        currentFrameVisibleVoxelBlockIdList_.reserve(size);
    }

    // 哈希表索引函数
    static int getHashIndex(Eigen::Vector3i voxelBlockPos) {
        return (((uint)voxelBlockPos(0) * 73856093u) ^ ((uint)voxelBlockPos(1) * 19349669u) ^
                ((uint)voxelBlockPos(2) * 83492791u)) &
               (uint)SDF_HASH_MASK;
    }

    // 获取体素块地址
    Voxel* get_voxelBolck(int id) { return voxel_ + id * SDF_BLOCK_SIZE3; }

    // 清除上一帧的可视体素块索引并交换上一帧和当前帧的数据指针以及其他数据
    void swapVisibleList() {
        lastFrametVisibleVoxelBlockList_.clear();
        lastFrametVisibleVoxelBlockList_.swap(currentFrameVisibleVoxelBlockIdList_);
    }

    // 设置哈希表entry
    void set_entry(HashEntry hashEntry, int id) { hashEntries_[id] = hashEntry; }

    // 获取相对的哈希表entry
    HashEntry& get_entry(int id) { return hashEntries_[id]; }

    void set_entryOffset(int id, int offset) { hashEntries_[id].offset = offset; }

    // 获取空闲的冲突列表所对应的Entry指针
    std::optional<HashEntry*> get_freeExcessEntry() {
        if (freeExcessEntries_.empty()) return std::nullopt;
        int id = freeExcessEntries_.back();
        freeExcessEntries_.pop_back();
        return &hashEntries_[id];
    }

    SceneParams get_sceneParams() const { return sceneParams_; }

    const std::vector<int>& get_constLastFrameVisibleVoxelBlockList() const {
        return lastFrametVisibleVoxelBlockList_;
    }

    std::vector<int>& get_lastFrameVisibleVoxelBlockList() {
        return lastFrametVisibleVoxelBlockList_;
    }

    const std::vector<int>& get_constCurrentFrameVisibleVoxelBlockList() const {
        return currentFrameVisibleVoxelBlockIdList_;
    }

    std::vector<int>& get_currentFrameVisibleVoxelBlockList() {
        return currentFrameVisibleVoxelBlockIdList_;
    }

private:
    Voxel* voxel_{nullptr};

    HashEntry* hashEntries_{nullptr};

    std::vector<int> freeExcessEntries_;

    SceneParams sceneParams_;

    std::vector<int> lastFrametVisibleVoxelBlockList_;

    std::vector<int> currentFrameVisibleVoxelBlockIdList_;
};
#endif
