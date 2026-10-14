#pragma once

#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Matrix3x3.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Math {

template <Numeric T>
struct Quaternion {
    T w{1};
    T x{0};
    T y{0};
    T z{0};

    constexpr Quaternion() noexcept = default;
    constexpr Quaternion(T w_, T x_, T y_, T z_) noexcept : w(w_), x(x_), y(y_), z(z_) {}

    static constexpr Quaternion Identity() noexcept {
        return { 1, 0, 0, 0 };
    }

    [[nodiscard]] constexpr Quaternion Conjugate() const noexcept {
        return { w, -x, -y, -z };
    }

    [[nodiscard]] constexpr T NormSq() const noexcept {
        return w * w + x * x + y * y + z * z;
    }

    [[nodiscard]] T Norm() const noexcept {
        return std::sqrt(NormSq());
    }

    [[nodiscard]] Quaternion Normalized() const noexcept {
        T n = Norm();
        if (n > static_cast<T>(1e-9)) {
            T inv = static_cast<T>(1) / n;
            return { w * inv, x * inv, y * inv, z * inv };
        }
        return Identity();
    }

    void Renormalize() noexcept {
        *this = Normalized();
    }

    // Hamilton product: q1 * q2
    [[nodiscard]] constexpr Quaternion operator*(const Quaternion& rhs) const noexcept {
        return {
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
        };
    }

    // Vector rotation: v' = q * v * q*
    [[nodiscard]] Vector3<T> Rotate(const Vector3<T>& v) const noexcept {
        Vector3<T> qv{ x, y, z };
        Vector3<T> t = static_cast<T>(2) * qv.Cross(v);
        return v + (w * t) + qv.Cross(t);
    }

    // Inverse vector rotation: v = q* * v' * q
    [[nodiscard]] Vector3<T> Unrotate(const Vector3<T>& v) const noexcept {
        return Conjugate().Rotate(v);
    }

    // Aerospace 3-2-1 convention: Yaw (psi around Z), Pitch (theta around Y), Roll (phi around X)
    [[nodiscard]] static Quaternion FromEuler(T yawRad, T pitchRad, T rollRad) noexcept {
        T cy = std::cos(yawRad * static_cast<T>(0.5));
        T sy = std::sin(yawRad * static_cast<T>(0.5));
        T cp = std::cos(pitchRad * static_cast<T>(0.5));
        T sp = std::sin(pitchRad * static_cast<T>(0.5));
        T cr = std::cos(rollRad * static_cast<T>(0.5));
        T sr = std::sin(rollRad * static_cast<T>(0.5));

        Quaternion q{
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
        return q.Normalized();
    }

    [[nodiscard]] static Quaternion FromAxisAngle(const Vector3<T>& axis, T angleRad) noexcept {
        T halfAngle = angleRad * static_cast<T>(0.5);
        T s = std::sin(halfAngle);
        Vector3<T> normAxis = axis.Normalized();
        return Quaternion(std::cos(halfAngle), normAxis.x * s, normAxis.y * s, normAxis.z * s).Normalized();
    }

    // Extract Euler angles: yaw (psi), pitch (theta), roll (phi)
    void ToEuler(T& outYawRad, T& outPitchRad, T& outRollRad) const noexcept {
        // Roll (x-axis rotation)
        T sinr_cosp = static_cast<T>(2) * (w * x + y * z);
        T cosr_cosp = static_cast<T>(1) - static_cast<T>(2) * (x * x + y * y);
        outRollRad = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation) with singularity clamp
        T sinp = static_cast<T>(2) * (w * y - z * x);
        if (std::abs(sinp) >= static_cast<T>(1)) {
            outPitchRad = std::copysign(static_cast<T>(1.5707963267948966), sinp); // +- pi/2
        } else {
            outPitchRad = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        T siny_cosp = static_cast<T>(2) * (w * z + x * y);
        T cosy_cosp = static_cast<T>(1) - static_cast<T>(2) * (y * y + z * z);
        outYawRad = std::atan2(siny_cosp, cosy_cosp);
    }

    [[nodiscard]] Matrix3x3<T> ToRotationMatrix() const noexcept {
        T xx = x * x;
        T yy = y * y;
        T zz = z * z;
        T xy = x * y;
        T xz = x * z;
        T yz = y * z;
        T wx = w * x;
        T wy = w * y;
        T wz = w * z;

        return Matrix3x3<T>(
            static_cast<T>(1) - static_cast<T>(2) * (yy + zz),
            static_cast<T>(2) * (xy - wz),
            static_cast<T>(2) * (xz + wy),

            static_cast<T>(2) * (xy + wz),
            static_cast<T>(1) - static_cast<T>(2) * (xx + zz),
            static_cast<T>(2) * (yz - wx),

            static_cast<T>(2) * (xz - wy),
            static_cast<T>(2) * (yz + wx),
            static_cast<T>(1) - static_cast<T>(2) * (xx + yy)
        );
    }

    [[nodiscard]] static Quaternion Slerp(const Quaternion& a, const Quaternion& b, T t) noexcept {
        T dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        Quaternion target = b;

        // Take shortest arc
        if (dot < static_cast<T>(0)) {
            dot = -dot;
            target = { -b.w, -b.x, -b.y, -b.z };
        }

        if (dot > static_cast<T>(0.9995)) {
            // Linear interpolation for near-parallel quaternions
            Quaternion q{
                a.w + (target.w - a.w) * t,
                a.x + (target.x - a.x) * t,
                a.y + (target.y - a.y) * t,
                a.z + (target.z - a.z) * t
            };
            return q.Normalized();
        }

        T theta_0 = std::acos(dot);
        T theta = theta_0 * t;
        T sin_theta = std::sin(theta);
        T sin_theta_0 = std::sin(theta_0);

        T s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        T s1 = sin_theta / sin_theta_0;

        return {
            s0 * a.w + s1 * target.w,
            s0 * a.x + s1 * target.x,
            s0 * a.y + s1 * target.y,
            s0 * a.z + s1 * target.z
        };
    }
};

using Quat4f = Quaternion<f32>;
using Quat4d = Quaternion<f64>;

} // namespace LokYS::Math
