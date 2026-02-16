#include "robosim/terrain/terrain.h"
#include "robosim/terrain/terrains/all_terrains.h"
#include <spdlog/spdlog.h>

namespace robosim::terrain {

TerrainFactory& TerrainFactory::instance() {
    static TerrainFactory inst;
    return inst;
}

void TerrainFactory::register_terrain(const std::string& name, Creator creator) {
    creators_[name] = std::move(creator);
}

std::unique_ptr<Terrain> TerrainFactory::create(const std::string& name) const {
    auto it = creators_.find(name);
    if (it == creators_.end()) {
        spdlog::warn("Unknown terrain type '{}', defaulting to flat", name);
        return std::make_unique<FlatTerrain>();
    }
    return it->second();
}

std::vector<std::string> TerrainFactory::available() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : creators_) names.push_back(name);
    return names;
}

// Register built-in terrains
namespace {
struct TerrainRegistrar {
    TerrainRegistrar() {
        auto& f = TerrainFactory::instance();
        f.register_terrain("flat", []() { return std::make_unique<FlatTerrain>(); });
        f.register_terrain("stairs", []() { return std::make_unique<StairsTerrain>(); });
        f.register_terrain("slope", []() { return std::make_unique<SlopeTerrain>(); });
        f.register_terrain("rough", []() { return std::make_unique<RoughTerrain>(); });
        f.register_terrain("stepping_stones", []() { return std::make_unique<SteppingStoneTerrain>(); });
    }
};
static TerrainRegistrar _terrain_registrar;
}

} // namespace robosim::terrain
