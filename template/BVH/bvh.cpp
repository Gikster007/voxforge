#include "precomp.h"

#include "bvh.h"

#define OGT_VOX_IMPLEMENTATION
#include "lib/ogt_vox.h"

void calculate_bounds(Box* voxel_objects, uint node_idx, BVHNode* pool, uint* indices)
{
    BVHNode& node = pool[node_idx];
    node.min = float3(1e30f);
    node.max = float3(-1e30f);

    for (uint first = node.first, i = 0; i < node.count; i++)
    {
        uint idx = indices[first + i];
        Box& leaf = voxel_objects[idx];
        node.min = fminf(node.min, leaf.min);
        node.max = fmaxf(node.max, leaf.max);
    }
}

void BVH::construct_bvh(Box* voxel_objects)
{
    // Create Index Array
    indices = new uint[N];
    for (int i = 0; i < N; i++)
        indices[i] = i;

    // Allocate BVH Root Node
    pool = new BVHNode[N * 2 - 1];
    root = &pool[0];
    pool_ptr = 2;

    // Subdivide Root Node
    root->first = 0;
    root->count = N;
    calculate_bounds(voxel_objects, root->first, pool, indices);
    root->subdivide(root->first, voxel_objects, pool, pool_ptr, indices, nodes_used);
}

void BVH::intersect_voxel(Ray& ray, Box& box)
{
    float3 b[2] = {box.min, box.max};
    // test if the ray intersects the box
    const int signx = ray.D.x < 0, signy = ray.D.y < 0, signz = ray.D.z < 0;
    float tmin = (b[signx].x - ray.O.x) * ray.rD.x;
    float tmax = (b[1 - signx].x - ray.O.x) * ray.rD.x;
    const float tymin = (b[signy].y - ray.O.y) * ray.rD.y;
    const float tymax = (b[1 - signy].y - ray.O.y) * ray.rD.y;
    if (tmin > tymax || tymin > tmax)
        goto miss;
    tmin = max(tmin, tymin), tmax = min(tmax, tymax);
    const float tzmin = (b[signz].z - ray.O.z) * ray.rD.z;
    const float tzmax = (b[1 - signz].z - ray.O.z) * ray.rD.z;
    if (tmin > tzmax || tzmin > tmax)
        goto miss;
    if ((tmin = max(tmin, tzmin)) > 0)
        if (ray.t < tmin)
            goto miss;
        else
        {
            ray.t = tmin;
            ray.steps++;
            //find_nearest(ray, box);
            return;
        }
miss:
    /*ray.t = 1e34f*/ return;
}

void BVH::intersect_bvh(Box* voxel_objects, Ray& ray, const uint node_idx)
{
    
    BVHNode& node = pool[node_idx];
    if (!intersect_aabb(ray, node.min, node.max))
        return;
    if (node.is_leaf())
    {
        for (uint i = 0; i < node.count; i++)
            intersect_voxel(ray, voxel_objects[indices[node.first + i]]);
    }
    else
    {
        intersect_bvh(voxel_objects, ray, node.left);
        intersect_bvh(voxel_objects, ray, node.left + 1);
    }
}

bool BVH::intersect_aabb(const Ray& ray, const float3 bmin, const float3 bmax)
{
    float tx1 = (bmin.x - ray.O.x) / ray.D.x, tx2 = (bmax.x - ray.O.x) / ray.D.x;
    float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
    float ty1 = (bmin.y - ray.O.y) / ray.D.y, ty2 = (bmax.y - ray.O.y) / ray.D.y;
    tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
    float tz1 = (bmin.z - ray.O.z) / ray.D.z, tz2 = (bmax.z - ray.O.z) / ray.D.z;
    tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
    return tmax >= tmin && tmin < ray.t && tmax > 0;
}

void BVHNode::subdivide(uint node_idx, Box* voxel_objects, BVHNode* pool, uint pool_ptr, uint* indices, uint& nodes_used)
{
    BVHNode& node = pool[node_idx];
    if (node.count < 3)
        return;

    // determine split axis and position
    float3 extent = node.max - node.min;
    int axis = 0;
    if (extent.y > extent.x)
        axis = 1;
    if (extent.z > extent[axis])
        axis = 2;
    float splitPos = node.min[axis] + extent[axis] * 0.5f;
    // in-place partition
    int i = node.first;
    int j = i + node.count - 1;
    while (i <= j)
    {
        if (voxel_objects[indices[i]].get_center()[axis] < splitPos)
            i++;
        else
            swap(indices[i], indices[j--]);
    }
    // abort split if one of the sides is empty
    int leftCount = i - node.first;
    if (leftCount == 0 || leftCount == node.count)
        return;
    // create child nodes
    int leftChildIdx = nodes_used++;
    int rightChildIdx = nodes_used++;
    pool[leftChildIdx].first = node.first;
    pool[leftChildIdx].count = leftCount;
    pool[rightChildIdx].first = i;
    pool[rightChildIdx].count = node.count - leftCount;
    node.left = leftChildIdx;
    node.count = 0;
    calculate_bounds(voxel_objects, leftChildIdx, pool, indices);
    calculate_bounds(voxel_objects, rightChildIdx, pool, indices);

    subdivide(leftChildIdx, voxel_objects, pool, pool_ptr, indices, nodes_used);
    subdivide(rightChildIdx, voxel_objects, pool, pool_ptr, indices, nodes_used);
}

bool BVH::setup_3ddda(const Ray& ray, DDAState& state, Box& box)
{
    // if ray is not inside the world: advance until it is
    state.t = 0;
    if (!box.contains(ray.O))
    {
        state.t = intersect_aabb(ray, box.min, box.max);
        if (state.t > 1e33f)
            return false; // ray misses voxel data entirely
    }

    // expressed in world space
    const float3 voxelMinBounds = box.min;
    const float3 voxelMaxBounds = box.max /*- box.min*/;

    /*const float3 voxelMinBounds = TransformPosition(box.min, box.model.matrix());
    const float3 voxelMaxBounds = TransformPosition(box.max, box.model.matrix());*/

    const float gridsizeFloat = static_cast<float>(box.size);
    const float cellSize = 1.0f / gridsizeFloat;
    state.step = make_int3(1 - ray.Dsign * 2);
    // based on our cube position
    const float3 posInGrid = gridsizeFloat * ((ray.O - voxelMinBounds) + (state.t + 0.00005f) * ray.D) / voxelMaxBounds;
    const float3 gridPlanes = (ceilf(posInGrid) - ray.Dsign) * cellSize;
    const int3 P = clamp(make_int3(posInGrid), 0, box.size - 1);
    state.X = P.x, state.Y = P.y, state.Z = P.z;
    state.tdelta = cellSize * float3(state.step) * ray.rD;
    state.tmax = ((gridPlanes * voxelMaxBounds) - (ray.O - voxelMinBounds)) * ray.rD;
    return true;
}

void BVH::find_nearest(Ray& ray, Box& box)
{
    // Save Initial Ray
    Ray initial_ray = ray;

    mat4 model_mat = box.model.matrix();
    mat4 inv_model_mat = model_mat.Inverted();

    // Transform the Ray
    ray.O = TransformPosition(ray.O, inv_model_mat);
    ray.D = TransformVector(ray.D, inv_model_mat);
    ray.rD = float3(1.0f / ray.D.x, 1.0f / ray.D.y, 1.0f / ray.D.z);
    ray.CalculateDsign();

    // Setup Amanatides & Woo Grid Traversal
    DDAState s;
    if (!setup_3ddda(ray, s, box))
        return;

    // Start Stepping
    while (s.t <= ray.t)
    {
        const uint8_t cell = box.grid[(s.Z * box.size * box.size) + (s.Y * box.size) + s.X];

        if (cell)
        {
            ray.t = s.t;
            ray.voxel = cell;

            // If the Ray Intersected With a Primitive Within The BVH
            // Correct the Normal Based on The Transformation
            //ray.N = normalize(TransformVector(ray.N, model_mat));
            break;
        }
        if (s.tmax.x < s.tmax.y)
        {
            if (s.tmax.x < s.tmax.z)
            {
                s.t = s.tmax.x, s.X += s.step.x;
                if (s.X >= box.size)
                    break;
                s.tmax.x += s.tdelta.x;
            }
            else
            {
                s.t = s.tmax.z, s.Z += s.step.z;
                if (s.Z >= box.size)
                    break;
                s.tmax.z += s.tdelta.z;
            }
        }
        else
        {
            if (s.tmax.y < s.tmax.z)
            {
                s.t = s.tmax.y, s.Y += s.step.y;
                if (s.Y >= box.size)
                    break;
                s.tmax.y += s.tdelta.y;
            }
            else
            {
                s.t = s.tmax.z, s.Z += s.step.z;
                if (s.Z >= box.size)
                    break;
                s.tmax.z += s.tdelta.z;
            }
        }
    }

    // Restore the original ray's transform
    ray.O = initial_ray.O;
    ray.D = initial_ray.D;
    ray.rD = initial_ray.rD;
    ray.Dsign = initial_ray.Dsign;
}

void Box::populate_grid()
{
    /* Load the model file */
    FILE* file;
    file = fopen("assets/lightsaber.vox", "rb");
    uint32_t buffer_size = _filelength(_fileno(file));
    uint8_t* buffer = new uint8_t[buffer_size];
    fread(buffer, buffer_size, 1, file);
    fclose(file);

    /* Parse the model file */
    const ogt_vox_scene* scene = ogt_vox_read_scene(buffer, buffer_size);
    delete[] buffer; /* Cleanup */

    /* Grab the first model in the scene */
    auto model = scene->models[0];
    printf("loaded model of size : %i, %i, %i\n", model->size_x, model->size_y, model->size_z);

    size = model->size_x;
    auto grid_size = model->size_x * model->size_y * model->size_z * sizeof(uint8_t);
    grid = (uint8_t*)MALLOC64(grid_size);
    memset(grid, 0, grid_size);

#pragma omp parallel for schedule(dynamic)
    for (int z = 0; z < model->size_z; z++)
    {
        for (int y = 0; y < model->size_y; y++)
        {
            for (int x = 0; x < model->size_x; x++)
            {
                const uint8_t voxel_index = model->voxel_data[(z * model->size_y * model->size_x) + ((model->size_y - y - 1) * model->size_x) + x];

                if (voxel_index != 0)
                {
                    printf("Voxel Index: %u \n", voxel_index);
                    // voxel_data[voxel_index].color.x = scene->palette.color[voxel_index].r / 255.0f;
                    // voxel_data[voxel_index].color.y = scene->palette.color[voxel_index].g / 255.0f;
                    // voxel_data[voxel_index].color.z = scene->palette.color[voxel_index].b / 255.0f;
                }

#if !AMD_CPU
                grid[morton_encode(floor(y / b), floor(z / b), floor(x / b))] = voxel_index == 0 ? 0 : 1;
#else
                grid[(z * model->size_y * model->size_x) + (y * model->size_x) + x] = voxel_index == 0 ? 0 : 1;
#endif
            }
        }
    }
}