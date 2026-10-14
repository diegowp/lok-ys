#pragma once

#include "LokYS/core/Types.hpp"
#include <cmath>

namespace LokYS::Math {

template <Numeric T>
struct Vector3 {
    T x{0};
    T y{0};
    T z{0};

    constexpr Vector3() noexcept = default;
    constexpr Vector3(T x_, T y_, T z_) noexcept : x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr Vector3 operator+(const Vector3& rhs) const noexcept {
        return { x + rhs.x, y + rhs.y, z + rhs.z };
    }
    [[nodiscard]] constexpr Vector3 operator-(const Vector3& rhs) const noexcept {
        return { x - rhs.x, y - rhs.y, z - rhs.z };
    }
    [[nodiscard]] constexpr Vector3 operator*(T scalar) const noexcept {
        return { x * scalar, y * scalar, z * scalar };
    }
    [[nodiscard]] constexpr Vector3 operator/(T scalar) const noexcept {
        T inv = static_cast<T>(1) / scalar;
        return { x * inv, y * inv, z * inv };
    }
    [[nodiscard]] constexpr Vector3 operator-() const noexcept {
        return { -x, -y, -z };
    }

    constexpr Vector3& operator+=(const Vector3& rhs) noexcept {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }
    constexpr Vector3& operator-=(const Vector3& rhs) noexcept {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }
    constexpr Vector3& operator*=(T scalar) noexcept {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }
    constexpr Vector3& operator/=(T scalar) noexcept {
        T inv = static_cast<T>(1) / scalar;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }

    [[nodiscard]] constexpr T Dot(const Vector3& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    [[nodiscard]] constexpr Vector3 Cross(const Vector3& rhs) const noexcept {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    [[nodiscard]] constexpr T LengthSq() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] T Length() const noexcept {
        return std::sqrt(LengthSq());
    }

    [[nodiscard]] Vector3 Normalized() const noexcept {
        T len = Length();
        if (len > static_cast<T>(1e-9)) {
            return *this / len;
        }
        return { 0, 0, 0 };
    }

    [[nodiscard]] static constexpr Vector3 Lerp(const Vector3& a, const Vector3& b, T t) noexcept {
        return a + (b - a) * t;
    }

    static constexpr Vector3 Zero() noexcept { return { 0, 0, 0 }; }
    static constexpr Vector3 UnitX() noexcept { return { 1, 0, 0 }; }
    static constexpr Vector3 UnitY() noexcept { return { 0, 1, 0 }; }
    static constexpr Vector3 UnitZ() noexcept { return { 0, 0, 1 }; }
};

template <Numeric T>
[[nodiscard]] constexpr Vector3<T> operator*(T scalar, const Vector3<T>& v) noexcept {
    return v * scalar;
}

using Vec3f = Vector3<f32>;
using Vec3d = Vector3<f64>;

template <Numeric T>
struct Vector4 {
    T x{0};
    T y{0};
    T z{0};
    T w{0};

    constexpr Vector4() noexcept = default;
    constexpr Vector4(T x_, T y_, T z_, T w_) noexcept : x(x_), y(y_), z(z_), w(w_) {}

    [[nodiscard]] constexpr Vector4 operator+(const Vector4& rhs) const noexcept {
        return { x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w };
    }
    [[nodiscard]] constexpr Vector4 operator-(const Vector4& rhs) const noexcept {
        return { x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w };
    }
    [[nodiscard]] constexpr Vector4 operator*(T scalar) const noexcept {
        return { x * scalar, y * scalar, z * scalar, w * scalar };
    }
};

using Vec4f = Vector4<f32>;
using Vec4d = Vector4<f64>;

} // namespace LokYS::Math

