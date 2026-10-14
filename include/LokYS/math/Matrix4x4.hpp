#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
#include <cmath>
#include <cstring>

namespace LokYS::Math {

struct Mat4f {
    // 4x4 matrix stored in row-major layout: m[row][col]
    f32 m[4][4]{
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 1.0f }
    };

    constexpr Mat4f() noexcept = default;

    constexpr Mat4f(
        f32 m00, f32 m01, f32 m02, f32 m03,
        f32 m10, f32 m11, f32 m12, f32 m13,
        f32 m20, f32 m21, f32 m22, f32 m23,
        f32 m30, f32 m31, f32 m32, f32 m33) noexcept
        : m{
            { m00, m01, m02, m03 },
            { m10, m11, m12, m13 },
            { m20, m21, m22, m23 },
            { m30, m31, m32, m33 }
        } {}

    [[nodiscard]] static constexpr Mat4f Identity() noexcept {
        return Mat4f{
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };
    }

    [[nodiscard]] static constexpr Mat4f Zero() noexcept {
        return Mat4f{
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f
        };
    }

    [[nodiscard]] static Mat4f Translation(const Vec3f& t) noexcept {
        return Mat4f{
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            t.x,  t.y,  t.z,  1.0f
        };
    }

    [[nodiscard]] static Mat4f Scale(const Vec3f& s) noexcept {
        return Mat4f{
            s.x,  0.0f, 0.0f, 0.0f,
            0.0f, s.y,  0.0f, 0.0f,
            0.0f, 0.0f, s.z,  0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };
    }

    [[nodiscard]] static Mat4f FromQuaternion(const Quat4d& q) noexcept {
        f64 xx = q.x * q.x;
        f64 yy = q.y * q.y;
        f64 zz = q.z * q.z;
        f64 xy = q.x * q.y;
        f64 xz = q.x * q.z;
        f64 yz = q.y * q.z;
        f64 wx = q.w * q.x;
        f64 wy = q.w * q.y;
        f64 wz = q.w * q.z;

        return Mat4f{
            static_cast<f32>(1.0 - 2.0 * (yy + zz)),
            static_cast<f32>(2.0 * (xy + wz)),
            static_cast<f32>(2.0 * (xz - wy)),
            0.0f,

            static_cast<f32>(2.0 * (xy - wz)),
            static_cast<f32>(1.0 - 2.0 * (xx + zz)),
            static_cast<f32>(2.0 * (yz + wx)),
            0.0f,

            static_cast<f32>(2.0 * (xz + wy)),
            static_cast<f32>(2.0 * (yz - wx)),
            static_cast<f32>(1.0 - 2.0 * (xx + yy)),
            0.0f,

            0.0f, 0.0f, 0.0f, 1.0f
        };
    }

    [[nodiscard]] static Mat4f FromQuaternion(const Quat4f& q) noexcept {
        f32 xx = q.x * q.x;
        f32 yy = q.y * q.y;
        f32 zz = q.z * q.z;
        f32 xy = q.x * q.y;
        f32 xz = q.x * q.z;
        f32 yz = q.y * q.z;
        f32 wx = q.w * q.x;
        f32 wy = q.w * q.y;
        f32 wz = q.w * q.z;

        return Mat4f{
            1.0f - 2.0f * (yy + zz),
            2.0f * (xy + wz),
            2.0f * (xz - wy),
            0.0f,

            2.0f * (xy - wz),
            1.0f - 2.0f * (xx + zz),
            2.0f * (yz + wx),
            0.0f,

            2.0f * (xz + wy),
            2.0f * (yz - wx),
            1.0f - 2.0f * (xx + yy),
            0.0f,

            0.0f, 0.0f, 0.0f, 1.0f
        };
    }

    [[nodiscard]] Mat4f operator*(const Mat4f& rhs) const noexcept {
        Mat4f res = Zero();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                res.m[r][c] = m[r][0] * rhs.m[0][c] +
                              m[r][1] * rhs.m[1][c] +
                              m[r][2] * rhs.m[2][c] +
                              m[r][3] * rhs.m[3][c];
            }
        }
        return res;
    }

    [[nodiscard]] Mat4f Transpose() const noexcept {
        return Mat4f{
            m[0][0], m[1][0], m[2][0], m[3][0],
            m[0][1], m[1][1], m[2][1], m[3][1],
            m[0][2], m[1][2], m[2][2], m[3][2],
            m[0][3], m[1][3], m[2][3], m[3][3]
        };
    }

    // Direct3D 12 standard Left-Handed Perspective Projection (Z in [0, 1])
    [[nodiscard]] static Mat4f PerspectiveFovLH(
        f32 fovYRadians,
        f32 aspectRatio,
        f32 nearZ,
        f32 farZ) noexcept
    {
        f32 h = 1.0f / std::tan(fovYRadians * 0.5f);
        f32 w = h / aspectRatio;
        f32 range = farZ - nearZ;

        return Mat4f{
            w,    0.0f, 0.0f,                   0.0f,
            0.0f, h,    0.0f,                   0.0f,
            0.0f, 0.0f, farZ / range,           1.0f,
            0.0f, 0.0f, -(nearZ * farZ) / range, 0.0f
        };
    }

    // Direct3D 12 Left-Handed LookAt View Matrix
    [[nodiscard]] static Mat4f LookAtLH(
        const Vec3f& eye,
        const Vec3f& target,
        const Vec3f& up) noexcept
    {
        Vec3f zAxis = (target - eye).Normalized();
        Vec3f xAxis = up.Cross(zAxis).Normalized();
        Vec3f yAxis = zAxis.Cross(xAxis);

        return Mat4f{
            xAxis.x, yAxis.x, zAxis.x, 0.0f,
            xAxis.y, yAxis.y, zAxis.y, 0.0f,
            xAxis.z, yAxis.z, zAxis.z, 0.0f,
            -eye.Dot(xAxis), -eye.Dot(yAxis), -eye.Dot(zAxis), 1.0f
        };
    }

    // Camera-relative View matrix (Eye is assumed at local origin [0, 0, 0])
    [[nodiscard]] static Mat4f CameraRelativeViewLH(
        const Vec3f& forward,
        const Vec3f& up) noexcept
    {
        Vec3f zAxis = forward.Normalized();
        Vec3f xAxis = up.Cross(zAxis).Normalized();
        Vec3f yAxis = zAxis.Cross(xAxis);

        return Mat4f{
            xAxis.x, yAxis.x, zAxis.x, 0.0f,
            xAxis.y, yAxis.y, zAxis.y, 0.0f,
            xAxis.z, yAxis.z, zAxis.z, 0.0f,
            0.0f,    0.0f,    0.0f,    1.0f
        };
    }

    // Direct3D 12 Left-Handed Orthographic Projection Matrix
    [[nodiscard]] static Mat4f OrthographicLH(
        f32 width,
        f32 height,
        f32 nearZ,
        f32 farZ) noexcept
    {
        f32 range = farZ - nearZ;
        return Mat4f{
            2.0f / width, 0.0f,          0.0f,         0.0f,
            0.0f,         2.0f / height, 0.0f,         0.0f,
            0.0f,         0.0f,          1.0f / range, 0.0f,
            0.0f,         0.0f,         -nearZ / range, 1.0f
        };
    }

    [[nodiscard]] static Mat4f OrthographicOffCenterLH(
        f32 left,
        f32 right,
        f32 bottom,
        f32 top,
        f32 nearZ,
        f32 farZ) noexcept
    {
        f32 w = right - left;
        f32 h = top - bottom;
        f32 d = farZ - nearZ;

        return Mat4f{
            2.0f / w,           0.0f,               0.0f,           0.0f,
            0.0f,               2.0f / h,           0.0f,           0.0f,
            0.0f,               0.0f,               1.0f / d,       0.0f,
            -(right + left) / w, -(top + bottom) / h, -nearZ / d,    1.0f
        };
    }
};

struct FrustumPlane {
    Vec3f normal{ 0.0f, 0.0f, 0.0f };
    f32 distance{ 0.0f };

    void Normalize() noexcept {
        f32 len = normal.Length();
        if (len > 1e-6f) {
            f32 inv = 1.0f / len;
            normal = normal * inv;
            distance *= inv;
        }
    }

    [[nodiscard]] f32 SignedDistance(const Vec3f& point) const noexcept {
        return normal.Dot(point) + distance;
    }
};

struct Frustum {
    FrustumPlane planes[6]; // Left, Right, Bottom, Top, Near, Far

    [[nodiscard]] static Frustum FromViewProj(const Mat4f& vp) noexcept {
        Frustum f;

        // Left plane: (vp._14 + vp._11, vp._24 + vp._21, vp._34 + vp._31, vp._44 + vp._41)
        f.planes[0].normal = { vp.m[0][3] + vp.m[0][0], vp.m[1][3] + vp.m[1][0], vp.m[2][3] + vp.m[2][0] };
        f.planes[0].distance = vp.m[3][3] + vp.m[3][0];

        // Right plane: (vp._14 - vp._11, vp._24 - vp._21, vp._34 - vp._31, vp._44 - vp._41)
        f.planes[1].normal = { vp.m[0][3] - vp.m[0][0], vp.m[1][3] - vp.m[1][0], vp.m[2][3] - vp.m[2][0] };
        f.planes[1].distance = vp.m[3][3] - vp.m[3][0];

        // Bottom plane: (vp._14 + vp._12, vp._24 + vp._21, vp._34 + vp._32, vp._44 + vp._42)
        f.planes[2].normal = { vp.m[0][3] + vp.m[0][1], vp.m[1][3] + vp.m[1][1], vp.m[2][3] + vp.m[2][1] };
        f.planes[2].distance = vp.m[3][3] + vp.m[3][1];

        // Top plane: (vp._14 - vp._12, vp._24 - vp._21, vp._34 - vp._32, vp._44 - vp._42)
        f.planes[3].normal = { vp.m[0][3] - vp.m[0][1], vp.m[1][3] - vp.m[1][1], vp.m[2][3] - vp.m[2][1] };
        f.planes[3].distance = vp.m[3][3] - vp.m[3][1];

        // Near plane: (vp._13, vp._23, vp._33, vp._43)
        f.planes[4].normal = { vp.m[0][2], vp.m[1][2], vp.m[2][2] };
        f.planes[4].distance = vp.m[3][2];

        // Far plane: (vp._14 - vp._13, vp._24 - vp._23, vp._34 - vp._33, vp._44 - vp._43)
        f.planes[5].normal = { vp.m[0][3] - vp.m[0][2], vp.m[1][3] - vp.m[1][2], vp.m[2][3] - vp.m[2][2] };
        f.planes[5].distance = vp.m[3][3] - vp.m[3][2];

        for (int i = 0; i < 6; ++i) {
            f.planes[i].Normalize();
        }

        return f;
    }

    [[nodiscard]] bool IntersectsSphere(const Vec3f& center, f32 radius) const noexcept {
        for (int i = 0; i < 6; ++i) {
            if (planes[i].SignedDistance(center) < -radius) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool IntersectsAABB(const Vec3f& minBounds, const Vec3f& maxBounds) const noexcept {
        for (int i = 0; i < 6; ++i) {
            // Find most positive vertex (p-vertex) along normal
            Vec3f p{
                (planes[i].normal.x >= 0.0f) ? maxBounds.x : minBounds.x,
                (planes[i].normal.y >= 0.0f) ? maxBounds.y : minBounds.y,
                (planes[i].normal.z >= 0.0f) ? maxBounds.z : minBounds.z
            };

            if (planes[i].SignedDistance(p) < 0.0f) {
                return false;
            }
        }
        return true;
    }
};

} // namespace LokYS::Math
