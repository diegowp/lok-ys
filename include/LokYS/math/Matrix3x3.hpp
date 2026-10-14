#pragma once

#include "LokYS/math/Vector3.hpp"

namespace LokYS::Math {

template <Numeric T>
struct Matrix3x3 {
    // Row-major order: m[row][col]
    T m[3][3]{
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 }
    };

    constexpr Matrix3x3() noexcept = default;

    constexpr Matrix3x3(
        T m00, T m01, T m02,
        T m10, T m11, T m12,
        T m20, T m21, T m22) noexcept
        : m{ {m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22} } {}

    static constexpr Matrix3x3 Identity() noexcept {
        return Matrix3x3();
    }

    static constexpr Matrix3x3 Zero() noexcept {
        return Matrix3x3(0,0,0, 0,0,0, 0,0,0);
    }

    [[nodiscard]] constexpr Matrix3x3 operator+(const Matrix3x3& b) const noexcept {
        return Matrix3x3(
            m[0][0]+b.m[0][0], m[0][1]+b.m[0][1], m[0][2]+b.m[0][2],
            m[1][0]+b.m[1][0], m[1][1]+b.m[1][1], m[1][2]+b.m[1][2],
            m[2][0]+b.m[2][0], m[2][1]+b.m[2][1], m[2][2]+b.m[2][2]
        );
    }

    [[nodiscard]] constexpr Matrix3x3 operator*(const Matrix3x3& b) const noexcept {
        Matrix3x3 result = Matrix3x3::Zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    result.m[i][j] += m[i][k] * b.m[k][j];
                }
            }
        }
        return result;
    }

    [[nodiscard]] constexpr Vector3<T> operator*(const Vector3<T>& v) const noexcept {
        return Vector3<T>(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    [[nodiscard]] constexpr Matrix3x3 Transposed() const noexcept {
        return Matrix3x3(
            m[0][0], m[1][0], m[2][0],
            m[0][1], m[1][1], m[2][1],
            m[0][2], m[1][2], m[2][2]
        );
    }

    [[nodiscard]] constexpr T Determinant() const noexcept {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    [[nodiscard]] Matrix3x3 Inverse() const noexcept {
        T det = Determinant();
        if (std::abs(det) < static_cast<T>(1e-9)) {
            return Matrix3x3::Identity();
        }
        T invDet = static_cast<T>(1) / det;

        return Matrix3x3(
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet,

            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet,

            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet
        );
    }
};

using Mat3f = Matrix3x3<f32>;
using Mat3d = Matrix3x3<f64>;

} // namespace LokYS::Math
