#pragma once

#include "Physics/BroadPhase/AABB.hpp"

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>
#include <limits>

class BVHTree2D
{
public:
    using ObjectHandle = uint32_t;
    // Low 32 bits store the node index; high 32 bits store its generation.
    using ProxyId = uint64_t;
    using NodeIndex = uint32_t;

    static constexpr ObjectHandle InvalidObjectHandle =
        std::numeric_limits<ObjectHandle>::max();
    static constexpr ProxyId InvalidIndex =
        std::numeric_limits<ProxyId>::max();
    
    ProxyId CreateProxy(ObjectHandle handle, const AABB2& bounds);
    // Updates the tight bounds; returns true when the fat leaf was reinserted.
    bool MoveProxy(ProxyId proxy, const AABB2& newBounds);
    bool DestroyProxy(ProxyId proxy);
    std::vector<ObjectHandle> Query(const AABB2 &range) const;
    // Appends matching handles to result; it does not clear the vector.
    void Query(const AABB2 &range, std::vector<ObjectHandle> &result) const;
    int GetHeight() const {
        return m_nodes[m_rootIndex].height;
    }

private:

    static constexpr NodeIndex InvalidNode =
        std::numeric_limits<NodeIndex>::max();

    struct BVHNode
    {
        AABB2 bounds{};      // Fat AABB2 used by the hierarchy.
        AABB2 tightBounds{}; // Actual object AABB2 used for exact query results.

        NodeIndex parent = InvalidNode;
        NodeIndex left = InvalidNode;
        NodeIndex right = InvalidNode;
        NodeIndex next = InvalidNode;

        ObjectHandle objectHandle = InvalidObjectHandle;
        uint32_t generation = 0;
        int height = 0;
        bool allocated = false;

        bool IsLeaf() const {
            return left == InvalidNode && right == InvalidNode;
        }
    };

    static NodeIndex NodeIndexFromProxy(ProxyId proxy) {
        return static_cast<NodeIndex>(proxy & 0xffffffffull);
    }

    static uint32_t GenerationFromProxy(ProxyId proxy) {
        return static_cast<uint32_t>(proxy >> 32);
    }

    static ProxyId MakeProxyId(NodeIndex node, uint32_t generation) {
        return (static_cast<ProxyId>(generation) << 32) | node;
    }

    bool IsValidLeaf(NodeIndex index) const {
        if (index == InvalidNode
            || index >= static_cast<std::size_t>(m_nodes.size())) return false;
        const auto& node = m_nodes[index];
        return node.allocated && node.IsLeaf()
            && node.objectHandle != InvalidObjectHandle;
    }

    void DetachLeaf(NodeIndex nodeIdx);
    void InsertLeaf(NodeIndex nodeIdx);
    NodeIndex FindBestSibling(NodeIndex nodeIdx);
    void RefitAndBalance(NodeIndex nodeIdx);
    NodeIndex Balance(NodeIndex nodeIdx);
    NodeIndex RotateRightChildUp(NodeIndex aIndex, NodeIndex cIndex);
    NodeIndex RotateLeftChildUp(NodeIndex aIndex, NodeIndex bIndex);
    void RecomputeNode(NodeIndex nodeIdx);

    bool IsValidProxy(ProxyId id) const {
        if (id == InvalidIndex) return false;
        const NodeIndex index = NodeIndexFromProxy(id);
        if (!IsValidLeaf(index)) return false;
        return m_nodes[index].generation == GenerationFromProxy(id);
    }

    NodeIndex AllocateNode() {
        if (m_freeNode != InvalidNode) {
            auto free = m_freeNode;
            auto& node = m_nodes[m_freeNode];
            m_freeNode = node.next;
            node.next = InvalidNode;
            node.allocated = true;
            return free;
        }

        BVHNode node;
        node.allocated = true;
        m_nodes.push_back(node);
        return static_cast<NodeIndex>(m_nodes.size() - 1);
    }

    void FreeNode(NodeIndex nodeIdx) {
        if (nodeIdx >= static_cast<std::size_t>(m_nodes.size())) return;
        auto& node = m_nodes[nodeIdx];
        const uint32_t nextGeneration = node.generation + 1;
        node = BVHNode{};
        node.generation = nextGeneration;
        node.next = m_freeNode;

        m_freeNode = nodeIdx;
    }

    AABB2 MakeFatBounds(const AABB2& bounds) const {
        return {
            {bounds.min.x - m_fatMargin, bounds.min.y - m_fatMargin},
            {bounds.max.x + m_fatMargin, bounds.max.y + m_fatMargin}
        };
    }

    NodeIndex m_rootIndex = InvalidNode;
    std::vector<BVHNode> m_nodes;
    NodeIndex m_freeNode = InvalidNode;

    float m_fatMargin = 0.1f;
};
