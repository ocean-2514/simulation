#include "Simulation/Cloth/ClothGrid.hpp"

std::shared_ptr<ClothGrid> ClothGrid::Create(
    const ClothGridDesc& desc) {
    if (desc.columns <= 1 || desc.rows <= 1) return nullptr;

    std::shared_ptr<ClothGrid> grid{new ClothGrid{}};
    grid->m_desc = desc;
    
    const uint32_t columns = desc.columns;
    const uint32_t rows = desc.rows;
    const float width = desc.width;
    const float height = desc.height;
    const float totalMass = desc.totalMass;
    const glm::vec3 origin = desc.origin;
    const float invColumns = 1.0f / (columns - 1);
    const float invRows = 1.0f / (rows - 1);
    const uint32_t particleCnt = columns * rows;
    const float particleMass = totalMass / particleCnt;
    const float invMass = 1.0f / particleMass;
    grid->m_particles.reserve(particleCnt);

    for (uint32_t row = 0; row < rows; ++row) {
        for (uint32_t col = 0; col < columns; ++col) {
            ClothParticle particle;
            float u = static_cast<float>(col) * invColumns;
            float v = static_cast<float>(row) * invRows;
            particle.invMass = invMass;
            particle.position.y = origin.y;
            particle.position.x = origin.x + (u - 0.5f) * width;
            particle.position.z = origin.z + (v - 0.5f) * height;
            particle.oldPosition = particle.position;
            grid->m_particles.push_back(particle);
        }
    }

    return grid;
}

void ClothGrid::BuildTriangleIndices() {
    const uint32_t columns = m_desc.columns;
    const uint32_t rows = m_desc.rows;
    const float width = m_desc.width;
    const float height = m_desc.height;

    m_triangleIndices.clear();
    m_triangleIndices.reserve(6 * (rows - 1) * (columns - 1));

    auto Index = [columns] (uint32_t row, uint32_t col) {
        return row * columns + col;
    };

    for (uint32_t row = 0; row < rows - 1; ++row) {
        for (uint32_t col = 0; col < columns - 1; ++col) {
            m_triangleIndices.push_back(Index(row, col));
            m_triangleIndices.push_back(Index(row + 1, col));
            m_triangleIndices.push_back(Index(row, col + 1));
            m_triangleIndices.push_back(Index(row, col + 1));
            m_triangleIndices.push_back(Index(row + 1, col));
            m_triangleIndices.push_back(Index(row + 1, col + 1));
        }
    }
}

void ClothGrid::PinParticle(uint32_t index) {
    if (index >= m_particles.size()) return;
    m_particles[index].isPinned = true;
    m_particles[index].invMass = 0.0f;
}

void ClothGrid::SetPinnedPosition(uint32_t index, const glm::vec3& position) {
    if (index >= m_particles.size()) return;
    m_particles[index].isPinned = true;
    m_particles[index].invMass = 0.0f;
    m_particles[index].position = position;
}
