#include "AxisAlignedBoundingBox.h"


namespace simox
{

    AxisAlignedBoundingBox::AxisAlignedBoundingBox()
    {
        setCenter({0, 0, 0});
        setExtents({0, 0, 0});
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max)
    {
        this->min() = min;
        this->max() = max;
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(
            const Eigen::Vector2f& limitsX, const Eigen::Vector2f& limitsY, const Eigen::Vector2f& limitsZ)
    {
        this->limitsX() = limitsX;
        this->limitsY() = limitsY;
        this->limitsZ() = limitsZ;
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
    {
        this->min() = Eigen::Vector3f(xMin, yMin, zMin);
        this->max() = Eigen::Vector3f(xMax, yMax, zMax);
    }

    AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Eigen::Matrix32f& limits) :
        _limits(limits)
    {
    }

    AxisAlignedBoundingBox AxisAlignedBoundingBox::Empty()
    {
        return AxisAlignedBoundingBox({0, 0, 0}, {0, 0, 0});
    }

    const Eigen::Matrix32f AxisAlignedBoundingBox::limits() const
    {
        return _limits;
    }

    Eigen::Matrix32f& AxisAlignedBoundingBox::limits()
    {
        return _limits;
    }

    void AxisAlignedBoundingBox::setLimits(const Eigen::Matrix32f& value)
    {
        this->_limits = value;
    }

    const Eigen::Vector3f AxisAlignedBoundingBox::min() const
    {
        return _limits.col(0);
    }

    Eigen::Matrix32f::ColXpr AxisAlignedBoundingBox::min()
    {
        return _limits.col(0);
    }

    void AxisAlignedBoundingBox::setMin(const Eigen::Vector3f& value)
    {
        min() = value;
    }

    const Eigen::Vector3f AxisAlignedBoundingBox::max() const
    {
        return _limits.col(1);
    }

    Eigen::Matrix32f::ColXpr AxisAlignedBoundingBox::max()
    {
        return _limits.col(1);
    }

    void AxisAlignedBoundingBox::setMax(const Eigen::Vector3f& value)
    {
        max() = value;
    }

    const Eigen::Vector3f AxisAlignedBoundingBox::center() const
    {
        return 0.5 * (min() + max());
    }

    void AxisAlignedBoundingBox::setCenter(const Eigen::Vector3f& value)
    {
        // Keep current extents, move center.
        const Eigen::Vector3f halfExtents = 0.5 * extents();
        min() = value - halfExtents;
        max() = value + halfExtents;
    }

    const Eigen::Vector3f AxisAlignedBoundingBox::extents() const
    {
        return max() - min();
    }

    void AxisAlignedBoundingBox::setExtents(const Eigen::Vector3f& value)
    {
        // Keep current center, move extents.
        const Eigen::Vector3f& _center = center();
        min() = _center - value/2;
        max() = _center + value/2;
    }

    const Eigen::Vector2f AxisAlignedBoundingBox::limitsX() const
    {
        return _limits.row(0);
    }

    Eigen::Matrix32f::RowXpr AxisAlignedBoundingBox::limitsX()
    {
        return _limits.row(0);
    }

    void AxisAlignedBoundingBox::setLimitsX(const Eigen::Vector2f& value)
    {
        limitsX() = value;
    }

    const Eigen::Vector2f AxisAlignedBoundingBox::limitsY() const
    {
        return _limits.row(1);
    }

    Eigen::Matrix32f::RowXpr AxisAlignedBoundingBox::limitsY()
    {
        return _limits.row(1);
    }

    void AxisAlignedBoundingBox::setLimitsY(const Eigen::Vector2f& value)
    {
        limitsY() = value;
    }

    const Eigen::Vector2f AxisAlignedBoundingBox::limitsZ() const
    {
        return _limits.row(2);
    }

    Eigen::Matrix32f::RowXpr AxisAlignedBoundingBox::limitsZ()
    {
        return _limits.row(2);
    }

    void AxisAlignedBoundingBox::setLimitsZ(const Eigen::Vector2f& value)
    {
        limitsZ() = value;
    }

    float AxisAlignedBoundingBox::minX() const
    {
        return _limits(0, 0);
    }

    float& AxisAlignedBoundingBox::minX()
    {
        return _limits(0, 0);
    }

    void AxisAlignedBoundingBox::setMinX(const float value)
    {
        maxX() = value;
    }

    float AxisAlignedBoundingBox::maxX() const
    {
        return _limits(0, 1);
    }

    float& AxisAlignedBoundingBox::maxX()
    {
        return _limits(0, 1);
    }

    void AxisAlignedBoundingBox::setMaxX(const float value)
    {
        maxY() = value;
    }

    float AxisAlignedBoundingBox::minY() const
    {
        return _limits(1, 0);
    }

    float& AxisAlignedBoundingBox::minY()
    {
        return _limits(1, 0);
    }

    void AxisAlignedBoundingBox::setMinY(const float value)
    {
        minY() = value;
    }

    float AxisAlignedBoundingBox::maxY() const
    {
        return _limits(1, 1);
    }

    float& AxisAlignedBoundingBox::maxY()
    {
        return _limits(1, 1);
    }

    void AxisAlignedBoundingBox::setMaxY(const float value)
    {
        maxY() = value;
    }

    float AxisAlignedBoundingBox::minZ() const
    {
        return _limits(2, 0);
    }

    float& AxisAlignedBoundingBox::minZ()
    {
        return _limits(2, 0);
    }

    void AxisAlignedBoundingBox::setMinZ(const float value)
    {
        minZ() = value;
    }

    float AxisAlignedBoundingBox::maxZ() const
    {
        return _limits(2, 1);
    }

    float& AxisAlignedBoundingBox::maxZ()
    {
        return _limits(2, 1);
    }

    void AxisAlignedBoundingBox::setMaxZ(const float value)
    {
        maxZ() = value;
    }

    bool AxisAlignedBoundingBox::empty() const
    {
        return extents().isZero();
    }

    float AxisAlignedBoundingBox::centralSquaredDistance(const AxisAlignedBoundingBox& other) const
    {
        return aabb::centralSquaredDistance(*this, other);
    }

    float AxisAlignedBoundingBox::centralDistance(const AxisAlignedBoundingBox& other) const
    {
        return aabb::centralDistance(*this, other);
    }

    bool AxisAlignedBoundingBox::isColliding(const AxisAlignedBoundingBox& b) const
    {
        return aabb::isColliding(*this, b);
    }

    float aabb::centralDistance(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs)
    {
        return (lhs.center() - rhs.center()).norm();
    }

    float aabb::centralSquaredDistance(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs)
    {
        return (lhs.center() - rhs.center()).squaredNorm();
    }

    bool aabb::isColliding(const AxisAlignedBoundingBox& a, const AxisAlignedBoundingBox& b)
    {
        return (a.minX() <= b.maxX() and a.maxX() >= b.minX() and
                a.minY() <= b.maxY() and a.maxY() >= b.minY() and
                a.minZ() <= b.maxZ() and a.maxZ() >= b.minZ());
    }

}

std::ostream& simox::operator<<(std::ostream& os, const AxisAlignedBoundingBox rhs)
{
    static const Eigen::IOFormat iof(3, 0, " ", "", "", "", "[", "]");
    return os << "AABB " << rhs.min().format(iof) << " to " << rhs.max().format(iof);
}
