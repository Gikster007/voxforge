#pragma once

constexpr int VOXELVOLUMES = 1; // Amount of Voxel Models in the game
constexpr int NOISESIZE = 128;

struct alignas(32) AABB
{
    float3 min = 1e34f;
    float3 max = -1e34f;

  private:
    float dummy1;
    float dummy2;

  public:
    AABB() = default;
    AABB(float3 _min, float3 _max)
    {
        min = _min;
        max = _max;
    }
    float3 get_center() const { return min + (max - min) * 0.5f; }
    void grow(const float3 p)
    {
        min = fminf(min, p);
        max = fmaxf(max, p);
    }
    void grow(const AABB& aabb)
    {
        if (aabb.min.x != 1e30f)
        {
            this->grow(aabb.min);
            this->grow(aabb.max);
        }
    }
    float area() const
    {
        const float3 e = max - min;
        return e.x * e.x + e.y * e.y + e.z * e.z;
    }
};

struct alignas(64) VoxelVolume
{
    uint8_t* grid;
    uint8_t* grids[GRIDLAYERS];
    int size;
    Transform model;
    float3 min = 1e34f, max = -1e34f;
    bool contains(const float3& pos) const
    {
        // test if pos is inside the cube
        AABB aabb = get_aabb();
        return pos.x >= aabb.min.x && pos.y >= aabb.min.y && pos.z >= aabb.min.z && pos.x <= aabb.max.x && pos.y <= aabb.max.y && pos.z <= aabb.max.z;
    }
    void populate_grid();
    AABB get_aabb() const // Credit to Max
    {
        float3 s = max - min; // size
        const float3 extent = s * 0.5f;
        mat4 mat = model.mat;

        /* Inspired by : <https://zeux.io/2010/10/17/aabb-from-obb-with-component-wise-abs/> */
        /* Get the transformed center and extent */
        const float3 t_center = TransformPosition(extent, mat);
        const float3 t_extent = TransformVector(extent, fabs(mat));

        return AABB(t_center - t_extent, t_center + t_extent);
    }
};

struct BVHNode
{
    bool is_leaf() const { return count > 0; }

#if AMD_CPU
    float3 min, max;
    uint left_first, count;
#else
    union {
        struct
        {
            float3 min;
            uint left_first;
        };
        __m128 min4;
    };
    union {
        struct
        {
            float3 max;
            uint count;
        };
        __m128 max4;
    };
#endif
};

class BVH
{
  public:
    void construct_bvh(VoxelVolume* voxel_objects);

    void intersect_bvh(VoxelVolume* voxel_objects, Ray& ray, const uint node_idx);

  private:
    float intersect_voxel_volume(Ray& ray, VoxelVolume& box);
#if !AMD_CPU
    float intersect_aabb(const Ray& ray, const float3 bmin, const float3 bmax);
#else
    float intersect_aabb_sse(const Ray& ray, const __m128 bmin4, const __m128 bmax4);
#endif

    bool setup_3ddda(const Ray& ray, DDAState& state, VoxelVolume& box);
    void find_nearest(Ray& ray, VoxelVolume& box, const int layer);

    float find_best_split_plane(VoxelVolume* voxel_objects, BVHNode& node, int& axis, float& pos) const;

    void subdivide(VoxelVolume* voxel_objects, BVHNode& node, int id);

  private:
    uint* indices = nullptr;
    BVHNode* pool = nullptr;
    BVHNode* root = nullptr;
    uint pool_ptr = 0;
    uint nodes_used = 2;
};