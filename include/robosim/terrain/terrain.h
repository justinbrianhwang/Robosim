#pragma once

#include "robosim/terrain/heightfield.h"
#include <string>
#include <memory>
#include <random>
#include <functional>
#include <unordered_map>

namespace robosim::terrain {

class Terrain {
public:
    virtual ~Terrain() = default;
    virtual Heightfield generate(std::mt19937& rng) const = 0;
    virtual std::string name() const = 0;
    virtual double friction() const { return 1.0; }
};

class TerrainFactory {
public:
    static TerrainFactory& instance();
    using Creator = std::function<std::unique_ptr<Terrain>()>;
    void register_terrain(const std::string& name, Creator creator);
    std::unique_ptr<Terrain> create(const std::string& name) const;
    std::vector<std::string> available() const;

private:
    std::unordered_map<std::string, Creator> creators_;
};

#define REGISTER_TERRAIN(cls, terrain_name) \
    namespace { static bool _reg_terrain_##cls = [] { \
        TerrainFactory::instance().register_terrain(terrain_name, \
            []() { return std::make_unique<cls>(); }); \
        return true; \
    }(); }

} // namespace robosim::terrain
