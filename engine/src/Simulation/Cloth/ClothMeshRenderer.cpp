#include "Simulation/Cloth/ClothMeshRenderer.hpp"

ClothMeshRenderer::ClothMeshRenderer(std::shared_ptr<ClothGrid> cloth) {
    if (!cloth) return;

    m_cloth = cloth;
    cloth->BuildTriangleIndices();
    indices = cloth->TriangleIndices();

    RenewVertices();
    SetUpBuffers();
}

void ClothMeshRenderer::RenewVertices() {
    if (!m_cloth) return;

    const uint32_t rows = m_cloth->Rows();
    const uint32_t columns = m_cloth->Columns();
    const auto& particles = m_cloth->Particles();
    
    vertices.clear();
    vertices.reserve(particles.size());

    for (std::size_t i = 0; i < particles.size(); ++i) {
        const auto& particle = particles[i];
        uint32_t row = static_cast<uint32_t>(i) / columns;
        uint32_t col = static_cast<uint32_t>(i) % columns;
        Vertex vertex;
        vertex.Position = particle.position;
        vertex.TexCoords = {
            static_cast<float>(col) / (columns - 1),
            static_cast<float>(row) / (rows - 1)
        };
        vertices.push_back(vertex);
    }

    for (std::size_t i = 0; i < indices.size(); i += 3) {
        unsigned int p0 = indices[i];
        unsigned int p1 = indices[i + 1];
        unsigned int p2 = indices[i + 2];
        glm::vec3 normal = glm::cross(
            particles[p1].position - particles[p0].position,
            particles[p2].position - particles[p0].position
        );
        vertices[p0].Normal += normal;
        vertices[p1].Normal += normal;
        vertices[p2].Normal += normal;
    }

    for (auto& vertex : vertices) {
        vertex.Normal = glm::length(vertex.Normal) <= 1e-6f 
            ? glm::vec3{1.0f, 0.0f, 0.0f}
            : glm::normalize(vertex.Normal);
    }

    if (VBO != 0) RenewVBO();
}