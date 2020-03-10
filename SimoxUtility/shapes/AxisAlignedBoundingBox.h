#pragma once

#include <Eigen/Core>


namespace Eigen
{
    using Matrix32f = Eigen::Matrix<float, 3, 2>;
}


namespace simox
{
    /**
     * @brief An axis-aligned (bounding-)box, defined by its limits in
     * x-, y- and z- direction.
     */
    struct AxisAlignedBoundingBox
    {
        /// Construct an AABB centered at (0, 0, 0) with extents (0, 0, 0).
        AxisAlignedBoundingBox();

        /// Construct an AABB with minimal and maximal values.
        AxisAlignedBoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max);
        /// Construct an AABB with axis-wise limits.
        AxisAlignedBoundingBox(const Eigen::Vector2f& limitsX, const Eigen::Vector2f& limitsY, const Eigen::Vector2f& limitsZ);
        /// Construct an AABB with given limits.
        AxisAlignedBoundingBox(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
        /// Construct an AABB with limits in a 3x2 matrix.
        AxisAlignedBoundingBox(const Eigen::Matrix32f& limits);

        /// Return an empty AABB (centered at zero with zero extents).
        static AxisAlignedBoundingBox Empty();


        const Eigen::Matrix32f limits() const;
        Eigen::Matrix32f& limits();
        void setLimits(const Eigen::Matrix32f& value);

        const Eigen::Vector3f min() const;
        Eigen::Matrix32f::ColXpr min();
        void setMin(const Eigen::Vector3f& value);

        const Eigen::Vector3f max() const;
        Eigen::Matrix32f::ColXpr max();
        void setMax(const Eigen::Vector3f& value);

        const Eigen::Vector3f center() const;
        void setCenter(const Eigen::Vector3f& value);
        const Eigen::Vector3f extents() const;
        void setExtents(const Eigen::Vector3f& value);

        const Eigen::Vector2f limitsX() const;
        Eigen::Matrix32f::RowXpr limitsX();
        void setLimitsX(const Eigen::Vector2f& value);

        const Eigen::Vector2f limitsY() const;
        Eigen::Matrix32f::RowXpr limitsY();
        void setLimitsY(const Eigen::Vector2f& value);

        const Eigen::Vector2f limitsZ() const;
        Eigen::Matrix32f::RowXpr limitsZ();
        void setLimitsZ(const Eigen::Vector2f& value);

        float minX() const;
        float& minX();
        void setMinX(const float value);
        float maxX() const;
        float& maxX();
        void setMaxX(const float value);

        float minY() const;
        float& minY();
        void setMinY(const float value);
        float maxY() const;
        float& maxY();
        void setMaxY(const float value);

        float minZ() const;
        float& minZ();
        void setMinZ(const float value);
        float maxZ() const;
        float& maxZ();
        void setMaxZ(const float value);

        /// Indicate whether this AABB is empty (i.e. has zero extents).
        bool empty() const;

        /// Return the distance between center of `*this` and `other`'s center.
        float centralDistance(const AxisAlignedBoundingBox& other) const;
        /// Return the squared distance between `*this` center and `other`'s center.
        float centralSquaredDistance(const AxisAlignedBoundingBox& other) const;

        /// Checks whether `*this` is colliding (i.e. overlapping) with `other`.
        bool isColliding(const AxisAlignedBoundingBox& other) const;


    private:

        /**
         * @brief The limits structured as follows:
         *
         * @code
         *    min    max
         * [ min_x  max_x ]  <- x
         * [ min_y  max_y ]  <- y
         * [ min_z  max_z ]  <- z
         * @endcode
         */
        Eigen::Matrix32f _limits = Eigen::Matrix32f::Zero();

    };


    std::ostream& operator<<(std::ostream& os, const AxisAlignedBoundingBox rhs);


namespace aabb
{
    /// Return the distance between center of `lhs`'s and `rhs`'s centers.
    float centralDistance(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs);

    /// Return the squared distance between `lhs`'s center and `rhs`'s centers.
    float centralSquaredDistance(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs);

    /// Checks whether `lhs` is colliding (i.e. overlapping) with `rhs`.
    bool isColliding(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs);
}

}
