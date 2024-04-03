#pragma once

constexpr int N = 4; // Amount of Voxel Models in the game

struct Box
{
    float3 min = 0.0f, max = 1.0f;
    uint8_t* grid;
    int size;
    Transform model;

    float3 get_center() const { return (max - min) * 0.5f; }
    bool contains(const float3& pos) const
    {
        // test if pos is inside the cube
        return pos.x >= min.x && pos.y >= min.y && pos.z >= min.z && pos.x <= max.x && pos.y <= max.y && pos.z <= max.z;
    }
    void populate_grid();
};

struct BVHNode
{
    void subdivide(uint node_idx, Box* voxel_objects, BVHNode* pool, uint pool_ptr, uint* indices, uint& nodes_used);
    bool is_leaf() const { return count > 0; }

    float3 min, max;
    uint left, first, count;
};

class BVH
{
  public:
    void construct_bvh(Box* voxel_objects);

    void intersect_voxel(Ray& ray, Box& box);

    void intersect_bvh(Box* voxel_objects, Ray& ray, const uint node_idx);
    bool intersect_aabb(const Ray& ray, const float3 bmin, const float3 bmax);

    bool setup_3ddda(const Ray& ray, DDAState& state, Box& box);
    void find_nearest(Ray& ray, Box& box);

    uint* indices;
    BVHNode* pool;
    BVHNode* root;
    uint pool_ptr;
    uint nodes_used = 1;
};