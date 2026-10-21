#include "Physics/BroadPhase/Spatial/BVHTree2D.hpp"
#include <algorithm>

BVHTree2D::ProxyId BVHTree2D::CreateProxy(
    ObjectHandle handle, const AABB2& bounds) {
    if (handle == InvalidObjectHandle || !bounds.IsValid()
        || m_nodes.size() >= static_cast<std::size_t>(InvalidNode)) {
        return InvalidIndex;
    }

    NodeIndex nodeIdx = AllocateNode();
    auto& node = m_nodes[nodeIdx];
    node.objectHandle = handle;
    node.tightBounds = bounds;
    node.bounds = MakeFatBounds(bounds);

    InsertLeaf(nodeIdx);
    return MakeProxyId(nodeIdx, m_nodes[nodeIdx].generation);
}

bool BVHTree2D::MoveProxy(ProxyId proxy, 
    const AABB2& newBounds) {
    if (!IsValidProxy(proxy) || !newBounds.IsValid()) return false;
    const NodeIndex nodeIdx = NodeIndexFromProxy(proxy);
    auto& node = m_nodes[nodeIdx];
    node.tightBounds = newBounds;
    if (node.bounds.Contains(newBounds)) {
        return false;
    }

    node.bounds = MakeFatBounds(newBounds);
    DetachLeaf(nodeIdx);
    InsertLeaf(nodeIdx);
    return true;
}

bool BVHTree2D::DestroyProxy(ProxyId proxy) {
    if (!IsValidProxy(proxy)) return false;
    const NodeIndex nodeIdx = NodeIndexFromProxy(proxy);
    DetachLeaf(nodeIdx);
    FreeNode(nodeIdx);
    return true;
}

std::vector<BVHTree2D::ObjectHandle> 
    BVHTree2D::Query(const AABB2 &range) const {
    std::vector<ObjectHandle> result;
    Query(range, result);
    return result;
}

void BVHTree2D::Query(const AABB2 &range, std::vector<ObjectHandle> &result) const {
    if (m_rootIndex == InvalidNode || !range.IsValid())
        return;

    std::vector<NodeIndex> stack;
    stack.push_back(m_rootIndex);
    while (!stack.empty())
    {
        const NodeIndex index = stack.back();
        stack.pop_back();
        const auto &node = m_nodes[index];

        if (!range.Overlaps(node.bounds))
            continue;

        if (node.IsLeaf())
        {
            if (range.Overlaps(node.tightBounds))
                result.push_back(node.objectHandle);
        }
        else
        {
            stack.push_back(node.left);
            stack.push_back(node.right);
        }
    }
}

void BVHTree2D::DetachLeaf(NodeIndex nodeIdx) {
    if (!IsValidLeaf(nodeIdx)) return;
    if (nodeIdx == m_rootIndex) {
        m_rootIndex = InvalidNode;
        m_nodes[nodeIdx].parent = InvalidNode;
        return;
    }

    auto& node = m_nodes[nodeIdx];
    NodeIndex parentIdx = node.parent;
    auto& parent = m_nodes[node.parent];
    bool isLeft = parent.left == nodeIdx;
    NodeIndex siblingIdx = isLeft ? parent.right : parent.left;
    auto& sibling = m_nodes[siblingIdx];
    if (parentIdx == m_rootIndex) {
        m_rootIndex = siblingIdx;
        sibling.parent = InvalidNode;
        FreeNode(parentIdx);
        node.parent = InvalidNode;
        return;
    }

    NodeIndex grandParentIdx = parent.parent;
    auto& grandParent = m_nodes[parent.parent];
    bool parentIsLeft = grandParent.left == parentIdx;
    sibling.parent = grandParentIdx;
    if (parentIsLeft) grandParent.left = siblingIdx;
    else grandParent.right = siblingIdx;
    FreeNode(parentIdx);
    node.parent = InvalidNode;

    RefitAndBalance(grandParentIdx);
}

void BVHTree2D::InsertLeaf(NodeIndex nodeIdx) {
    if (!IsValidLeaf(nodeIdx)) return;
    if (m_rootIndex == InvalidNode) {
        m_rootIndex = nodeIdx;
        m_nodes[nodeIdx].parent = InvalidNode;
        return;
    }

    NodeIndex newParentIdx = AllocateNode();
    auto& node = m_nodes[nodeIdx];
    NodeIndex siblingIdx = FindBestSibling(nodeIdx);
    auto& sibling = m_nodes[siblingIdx];
    if (siblingIdx == m_rootIndex) {
        auto& parent = m_nodes[newParentIdx];
        parent.left = siblingIdx;
        parent.right = nodeIdx;
        parent.parent = InvalidNode;
        m_rootIndex = newParentIdx;
        sibling.parent = node.parent = newParentIdx;
    } else {
        NodeIndex oldParentIdx = sibling.parent;
        auto& oldParent = m_nodes[oldParentIdx];
        auto& newParent = m_nodes[newParentIdx];
        bool isLeft = oldParent.left == siblingIdx;
        if (isLeft) oldParent.left = newParentIdx;
        else oldParent.right = newParentIdx;
        newParent.parent = oldParentIdx;
        newParent.left = siblingIdx;
        newParent.right = nodeIdx;
        sibling.parent = node.parent = newParentIdx;
    }
    RefitAndBalance(nodeIdx);
}

BVHTree2D::NodeIndex BVHTree2D::FindBestSibling(NodeIndex nodeIdx) {
    if (m_rootIndex == InvalidNode) return InvalidNode;

    const auto& leaf = m_nodes[nodeIdx];
    NodeIndex current = m_rootIndex;
    while (!m_nodes[current].IsLeaf()) {
        const auto& currentNode = m_nodes[current];
        const auto& left = m_nodes[currentNode.left];
        const auto& right = m_nodes[currentNode.right];
        const float combinedArea = AABB2::Merge(
            leaf.bounds, currentNode.bounds).Area();
        const float inheritedCost = 2.0f
            * (combinedArea - currentNode.bounds.Area());
        const float parentCost = 2.0f * combinedArea;

        const float leftCombinedArea = AABB2::Merge(
            leaf.bounds, left.bounds).Area();
        const float leftCost = (left.IsLeaf()
            ? leftCombinedArea
            : leftCombinedArea - left.bounds.Area()) + inheritedCost;

        const float rightCombinedArea = AABB2::Merge(
            leaf.bounds, right.bounds).Area();
        const float rightCost = (right.IsLeaf()
            ? rightCombinedArea
            : rightCombinedArea - right.bounds.Area()) + inheritedCost;

        if (parentCost <= leftCost && parentCost <= rightCost) {
            break;
        }
        current = leftCost < rightCost
            ? currentNode.left
            : currentNode.right;
    }

    return current;
}

void BVHTree2D::RefitAndBalance(NodeIndex nodeIdx) {
    NodeIndex currentIdx = nodeIdx;
    while (currentIdx != InvalidNode) {
        currentIdx = Balance(currentIdx);
        auto& node = m_nodes[currentIdx];
        if (!node.IsLeaf()) {
            RecomputeNode(currentIdx);
        }
        currentIdx = node.parent;
    }
}

BVHTree2D::NodeIndex BVHTree2D::Balance(NodeIndex nodeIdx) {
    auto& node = m_nodes[nodeIdx];
    if (node.IsLeaf()) {
        return nodeIdx;
    }

    NodeIndex leftIdx = node.left;
    NodeIndex rightIdx = node.right;
    auto& left = m_nodes[leftIdx];
    auto& right = m_nodes[rightIdx];
    int height = right.height - left.height;
    if (height > 1) {
        return RotateRightChildUp(nodeIdx, rightIdx);
    } else if (height < -1) {
        return RotateLeftChildUp(nodeIdx, leftIdx);
    }
    return nodeIdx;
}

BVHTree2D::NodeIndex BVHTree2D::RotateRightChildUp(
    NodeIndex aIndex, NodeIndex cIndex) {
    auto& nodeA = m_nodes[aIndex];
    if (m_nodes[m_nodes[cIndex].left].height > m_nodes[m_nodes[cIndex].right].height) {
        cIndex = RotateLeftChildUp(cIndex, m_nodes[cIndex].left);
    }

    auto& nodeC = m_nodes[cIndex];
    NodeIndex parentIdx = nodeA.parent;
    NodeIndex nodeCLeft = nodeC.left;
    nodeA.parent = cIndex;
    nodeA.right = nodeC.left;
    nodeC.parent = parentIdx;
    nodeC.left = aIndex;
    m_nodes[nodeCLeft].parent = aIndex;
    
    if (parentIdx == InvalidNode) m_rootIndex = cIndex;
    else {
        auto& parent = m_nodes[parentIdx];
        bool isLeft = parent.left == aIndex;
        if (isLeft) parent.left = cIndex;
        else parent.right = cIndex;
    }

    RecomputeNode(aIndex);
    RecomputeNode(cIndex);

    return cIndex;
}

BVHTree2D::NodeIndex BVHTree2D::RotateLeftChildUp(
    NodeIndex aIndex, NodeIndex bIndex) {
    auto& nodeA = m_nodes[aIndex];
    if (m_nodes[m_nodes[bIndex].left].height < m_nodes[m_nodes[bIndex].right].height) {
        bIndex = RotateRightChildUp(bIndex, m_nodes[bIndex].right);
    }

    auto& nodeB = m_nodes[bIndex];
    NodeIndex parentIdx = nodeA.parent;
    NodeIndex nodeBRight = nodeB.right;
    nodeA.parent = bIndex;
    nodeA.left = nodeB.right;
    nodeB.parent = parentIdx;
    nodeB.right = aIndex;
    m_nodes[nodeBRight].parent = aIndex;
    
    if (parentIdx == InvalidNode) m_rootIndex = bIndex;
    else {
        auto& parent = m_nodes[parentIdx];
        bool isLeft = parent.left == aIndex;
        if (isLeft) parent.left = bIndex;
        else parent.right = bIndex;
    }

    RecomputeNode(aIndex);
    RecomputeNode(bIndex);

    return bIndex;
}   

void BVHTree2D::RecomputeNode(NodeIndex nodeIdx) {
    auto& node = m_nodes[nodeIdx];
    if (node.IsLeaf()) return;

    auto& left = m_nodes[node.left];
    auto& right = m_nodes[node.right];
    node.height = 1 + std::max(left.height, right.height);
    node.bounds = AABB2::Merge(left.bounds, right.bounds);
}
