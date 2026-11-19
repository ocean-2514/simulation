#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>
#include <memory>

#include "Simulation/Cloth/ClothParticle.hpp"

struct ClothGridDesc {
    uint32_t columns = 16;
    uint32_t rows = 16;
    float width = 2.0f;
    float height = 2.0f;
    float totalMass = 1.0f;
    glm::vec3 origin{0.0f};
};

class ClothGrid {
public:
    static std::shared_ptr<ClothGrid> Create(const ClothGridDesc& desc);

    uint32_t Columns() const { return m_desc.columns; }
    uint32_t Rows() const { return m_desc.rows; }
    float SpacingX() const { return m_desc.width / (m_desc.columns - 1); }
    float SpacingY() const { return m_desc.height / (m_desc.rows - 1); }
    std::vector<ClothParticle>& Particles() { return m_particles; }
    const std::vector<ClothParticle>& Particles() const { return m_particles; }
    const std::vector<uint32_t>& TriangleIndices() const { return m_triangleIndices; }

    void BuildTriangleIndices();
    
    void PinParticle(uint32_t index);
    void SetPinnedPosition(uint32_t index, const glm::vec3& position);

private:
    ClothGrid() = default;
    
    ClothGridDesc m_desc;
    std::vector<ClothParticle> m_particles;
    std::vector<uint32_t> m_triangleIndices;
};