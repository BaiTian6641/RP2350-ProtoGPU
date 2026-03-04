/**
 * @file pgl_math.cpp
 * @brief GPU-side math primitives — implementation.
 *
 * Uses the RP2350 Cortex-M33 hardware FPU (single-precision).
 * All operations are branchless where possible for pipeline friendliness.
 */

#include "pgl_math.h"
#include <cmath>

namespace PglMath {

// ─── Vector 3D ──────────────────────────────────────────────────────────────

PglVec3 Add(const PglVec3& a, const PglVec3& b) {
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

PglVec3 Sub(const PglVec3& a, const PglVec3& b) {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

PglVec3 Mul(const PglVec3& v, float s) {
    return { v.x * s, v.y * s, v.z * s };
}

float Dot(const PglVec3& a, const PglVec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

PglVec3 Cross(const PglVec3& a, const PglVec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float Length(const PglVec3& v) {
    return sqrtf(Dot(v, v));
}

PglVec3 Normalize(const PglVec3& v) {
    float len = Length(v);
    if (len < 1e-8f) return { 0.0f, 0.0f, 0.0f };
    float inv = 1.0f / len;
    return { v.x * inv, v.y * inv, v.z * inv };
}

// ─── Vector 2D ──────────────────────────────────────────────────────────────

PglVec2 Add2(const PglVec2& a, const PglVec2& b) {
    return { a.x + b.x, a.y + b.y };
}

PglVec2 Sub2(const PglVec2& a, const PglVec2& b) {
    return { a.x - b.x, a.y - b.y };
}

PglVec2 Mul2(const PglVec2& v, float s) {
    return { v.x * s, v.y * s };
}

float Dot2(const PglVec2& a, const PglVec2& b) {
    return a.x * b.x + a.y * b.y;
}

// ─── Quaternion ─────────────────────────────────────────────────────────────

PglQuat QuatMul(const PglQuat& a, const PglQuat& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

PglQuat QuatConjugate(const PglQuat& q) {
    return { q.w, -q.x, -q.y, -q.z };
}

PglVec3 QuatRotate(const PglQuat& q, const PglVec3& v) {
    // q * (0,v) * q*  — expanded to avoid intermediate PglQuat
    float tx = 2.0f * (q.y * v.z - q.z * v.y);
    float ty = 2.0f * (q.z * v.x - q.x * v.z);
    float tz = 2.0f * (q.x * v.y - q.y * v.x);
    return {
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx)
    };
}

PglQuat QuatNormalize(const PglQuat& q) {
    float len = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (len < 1e-8f) return { 1.0f, 0.0f, 0.0f, 0.0f };
    float inv = 1.0f / len;
    return { q.w * inv, q.x * inv, q.y * inv, q.z * inv };
}

PglQuat QuatSlerp(const PglQuat& a, const PglQuat& b, float t) {
    // Compute 4D dot product
    float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

    // Handle double-cover: if dot < 0, negate one quaternion
    PglQuat b2 = b;
    if (dot < 0.0f) {
        dot = -dot;
        b2 = { -b.w, -b.x, -b.y, -b.z };
    }

    // Clamp to avoid acos domain errors from float imprecision
    if (dot > 1.0f) dot = 1.0f;

    float angle = acosf(dot);
    float sinAngle = sinf(angle);

    // If angle is very small, fall back to normalized lerp
    if (sinAngle < 1e-6f) {
        return QuatNormalize({
            a.w + (b2.w - a.w) * t,
            a.x + (b2.x - a.x) * t,
            a.y + (b2.y - a.y) * t,
            a.z + (b2.z - a.z) * t
        });
    }

    float invSin = 1.0f / sinAngle;
    float wa = sinf((1.0f - t) * angle) * invSin;
    float wb = sinf(t * angle) * invSin;

    return {
        wa * a.w + wb * b2.w,
        wa * a.x + wb * b2.x,
        wa * a.y + wb * b2.y,
        wa * a.z + wb * b2.z
    };
}

bool QuatIsClose(const PglQuat& a, const PglQuat& b, float epsilon) {
    // 4D dot product — accounts for double-cover (q ≡ -q)
    float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    return fabsf(dot) > (1.0f - epsilon);
}

// ─── Rotation Matrix ────────────────────────────────────────────────────────

Mat3 QuatToMat3(const PglQuat& q) {
    float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    float xx = q.x * x2,   xy = q.x * y2,   xz = q.x * z2;
    float yy = q.y * y2,   yz = q.y * z2,   zz = q.z * z2;
    float wx = q.w * x2,   wy = q.w * y2,   wz = q.w * z2;

    Mat3 m;
    m.m[0] = 1.0f - (yy + zz);  m.m[1] = xy - wz;            m.m[2] = xz + wy;
    m.m[3] = xy + wz;            m.m[4] = 1.0f - (xx + zz);   m.m[5] = yz - wx;
    m.m[6] = xz - wy;            m.m[7] = yz + wx;             m.m[8] = 1.0f - (xx + yy);
    return m;
}

PglVec3 Mat3MulVec(const Mat3& m, const PglVec3& v) {
    return {
        m.m[0] * v.x + m.m[1] * v.y + m.m[2] * v.z,
        m.m[3] * v.x + m.m[4] * v.y + m.m[5] * v.z,
        m.m[6] * v.x + m.m[7] * v.y + m.m[8] * v.z
    };
}

// ─── Transform ──────────────────────────────────────────────────────────────

PglVec3 TransformVertex(const PglTransform& t, const PglVec3& v) {
    // 1. Apply scale offset
    PglVec3 offset = Sub(v, t.scaleOffset);

    // 2. Apply scale-rotation offset
    PglVec3 scaled = {
        offset.x * t.scale.x,
        offset.y * t.scale.y,
        offset.z * t.scale.z
    };

    PglVec3 rotated = QuatRotate(t.scaleRotationOffset, scaled);
    PglVec3 reoffset = Add(rotated, t.scaleOffset);

    // 3. Apply rotation offset, main rotation, then base rotation
    PglVec3 rOff = Sub(reoffset, t.rotationOffset);
    PglQuat fullRot = QuatMul(t.rotation, t.baseRotation);
    PglVec3 finalRot = QuatRotate(fullRot, rOff);
    PglVec3 result = Add(finalRot, t.rotationOffset);

    // 4. Translate
    return Add(result, t.position);
}

// ─── Projection ─────────────────────────────────────────────────────────────

PglVec2 PerspectiveProject(const PglVec3& worldPos,
                           const PglVec3& camPos,
                           const PglQuat& camRot,
                           float fovFactor,
                           float screenW, float screenH,
                           float* outZ) {
    // Camera-relative position
    PglVec3 rel = Sub(worldPos, camPos);
    PglVec3 view = QuatRotate(QuatConjugate(camRot), rel);

    float z = view.z;
    if (z < 0.001f) z = 0.001f;  // clamp to avoid div-by-zero
    if (outZ) *outZ = z;

    float invZ = fovFactor / z;
    return {
        view.x * invZ + screenW * 0.5f,
        view.y * invZ + screenH * 0.5f
    };
}

PglVec2 OrthoProject(const PglVec3& worldPos,
                     const PglVec3& camPos,
                     float screenW, float screenH) {
    return {
        (worldPos.x - camPos.x) + screenW * 0.5f,
        (worldPos.y - camPos.y) + screenH * 0.5f
    };
}

// ─── Triangle Utilities ─────────────────────────────────────────────────────

PglVec3 TriangleNormal(const PglVec3& a, const PglVec3& b, const PglVec3& c) {
    return Cross(Sub(b, a), Sub(c, a));
}

float TriangleArea2D(const PglVec2& a, const PglVec2& b, const PglVec2& c) {
    return 0.5f * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
}

AABB2D TriangleBounds2D(const PglVec2& a, const PglVec2& b, const PglVec2& c) {
    AABB2D box;
    box.minX = fminf(a.x, fminf(b.x, c.x));
    box.minY = fminf(a.y, fminf(b.y, c.y));
    box.maxX = fmaxf(a.x, fmaxf(b.x, c.x));
    box.maxY = fmaxf(a.y, fmaxf(b.y, c.y));
    return box;
}

// ─── Misc ───────────────────────────────────────────────────────────────────

float Clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float Lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

int Min(int a, int b) { return a < b ? a : b; }
int Max(int a, int b) { return a > b ? a : b; }

}  // namespace PglMath
