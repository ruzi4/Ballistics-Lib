#pragma once

#include <cmath>
#include <ostream>

namespace ballistics {

/// Lightweight 3-D vector using double precision.
/// All operations are inlined for zero-overhead in hot simulation loops.
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() noexcept = default;
    constexpr Vec3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    // -----------------------------------------------------------------------
    // Arithmetic
    // -----------------------------------------------------------------------
    [[nodiscard]] constexpr Vec3 operator+(const Vec3& o) const noexcept {
        return {x + o.x, y + o.y, z + o.z};
    }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& o) const noexcept {
        return {x - o.x, y - o.y, z - o.z};
    }
    [[nodiscard]] constexpr Vec3 operator*(double s) const noexcept {
        return {x * s, y * s, z * s};
    }
    [[nodiscard]] constexpr Vec3 operator/(double s) const noexcept {
        return {x / s, y / s, z / s};
    }
    [[nodiscard]] constexpr Vec3 operator-() const noexcept {
        return {-x, -y, -z};
    }

    constexpr Vec3& operator+=(const Vec3& o) noexcept {
        x += o.x; y += o.y; z += o.z; return *this;
    }
    constexpr Vec3& operator-=(const Vec3& o) noexcept {
        x -= o.x; y -= o.y; z -= o.z; return *this;
    }
    constexpr Vec3& operator*=(double s) noexcept {
        x *= s; y *= s; z *= s; return *this;
    }

    // -----------------------------------------------------------------------
    // Products & norms
    // -----------------------------------------------------------------------
    [[nodiscard]] constexpr double dot(const Vec3& o) const noexcept {
        return x * o.x + y * o.y + z * o.z;
    }
    [[nodiscard]] constexpr Vec3 cross(const Vec3& o) const noexcept {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }
    [[nodiscard]] constexpr double norm_sq() const noexcept {
        return x * x + y * y + z * z;
    }
    [[nodiscard]] double norm() const noexcept {
        return std::sqrt(norm_sq());
    }
    /// Return a unit vector in the same direction.
    /// @note  Returns Vec3{0,0,0} (the zero vector) when called on a zero-length
    ///        vector.  Callers must check for this if a zero-velocity input is
    ///        possible (e.g. a projectile at rest).
    [[nodiscard]] Vec3 normalized() const noexcept {
        const double n = norm();
        return (n > 0.0) ? (*this / n) : Vec3{};
    }
};

// Scalar * Vec3 commutative form
[[nodiscard]] inline constexpr Vec3 operator*(double s, const Vec3& v) noexcept {
    return v * s;
}

inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    return os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
}

} // namespace ballistics
