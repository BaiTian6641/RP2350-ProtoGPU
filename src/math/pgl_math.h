/**
 * @file pgl_math.h
 * @brief GPU-side math primitives — Vector3D, Vector2D, Quaternion, Matrix.
 *
 * Lightweight versions of ProtoTracer's math types, operating directly on
 * PglVec2 / PglVec3 / PglQuat wire-format structs to avoid copies.
 *
 * These are not full replacements for the host math library; they cover only
 * the operations the rasterizer needs:
 *   - Transform a vertex by a PglTransform
 *   - Project 3D → 2D (perspective or orthographic)
 *   - Quaternion → rotation matrix
 *   - Triangle normal, area, 2D bounding box
 */

#pragma once

#include <PglTypes.h>
#include <cstdint>

namespace PglMath {

// ─── Vector Operations ──────────────────────────────────────────────────────

PglVec3 Add(const PglVec3& a, const PglVec3& b);
PglVec3 Sub(const PglVec3& a, const PglVec3& b);
PglVec3 Mul(const PglVec3& v, float s);
float   Dot(const PglVec3& a, const PglVec3& b);
PglVec3 Cross(const PglVec3& a, const PglVec3& b);
float   Length(const PglVec3& v);
PglVec3 Normalize(const PglVec3& v);

PglVec2 Add2(const PglVec2& a, const PglVec2& b);
PglVec2 Sub2(const PglVec2& a, const PglVec2& b);
PglVec2 Mul2(const PglVec2& v, float s);
float   Dot2(const PglVec2& a, const PglVec2& b);

// ─── Quaternion Operations ──────────────────────────────────────────────────

PglQuat QuatMul(const PglQuat& a, const PglQuat& b);
PglQuat QuatConjugate(const PglQuat& q);
PglVec3 QuatRotate(const PglQuat& q, const PglVec3& v);
PglQuat QuatNormalize(const PglQuat& q);

/// Spherical linear interpolation between two quaternions.
/// Handles double-cover (negates b if dot < 0). Falls back to Lerp+normalize
/// when the angle is very small to avoid division by near-zero sin.
PglQuat QuatSlerp(const PglQuat& a, const PglQuat& b, float t);

/// Returns true if two quaternions represent approximately the same rotation.
/// Accounts for quaternion double-cover (q and -q are the same rotation).
bool QuatIsClose(const PglQuat& a, const PglQuat& b, float epsilon = 0.001f);

// ─── 3×3 Rotation Matrix (computed from quaternion) ─────────────────────────

struct Mat3 {
    float m[9];  // row-major: m[row*3 + col]
};

Mat3    QuatToMat3(const PglQuat& q);
PglVec3 Mat3MulVec(const Mat3& m, const PglVec3& v);

// ─── Transform ──────────────────────────────────────────────────────────────

/// Apply a full PglTransform to a vertex: rotation, scale, translation.
/// Mirrors Transform::GetTransformMatrix() from ProtoTracer.
PglVec3 TransformVertex(const PglTransform& t, const PglVec3& v);

// ─── Projection ─────────────────────────────────────────────────────────────

/// Perspective project: returns screen-space XY.  Z is stored in *outZ.
PglVec2 PerspectiveProject(const PglVec3& worldPos,
                           const PglVec3& camPos,
                           const PglQuat& camRot,
                           float fovFactor,
                           float screenW, float screenH,
                           float* outZ);

/// Orthographic project for 2D cameras.
PglVec2 OrthoProject(const PglVec3& worldPos,
                     const PglVec3& camPos,
                     float screenW, float screenH);

// ─── Triangle Utilities ─────────────────────────────────────────────────────

/// Face normal of a triangle (un-normalized).
PglVec3 TriangleNormal(const PglVec3& a, const PglVec3& b, const PglVec3& c);

/// Signed 2D area (for winding / back-face culling).
float TriangleArea2D(const PglVec2& a, const PglVec2& b, const PglVec2& c);

/// Axis-aligned bounding box of a 2D triangle.
struct AABB2D {
    float minX, minY, maxX, maxY;
};
AABB2D TriangleBounds2D(const PglVec2& a, const PglVec2& b, const PglVec2& c);

// ─── Misc ───────────────────────────────────────────────────────────────────

float Clamp(float v, float lo, float hi);
float Lerp(float a, float b, float t);
int   Min(int a, int b);
int   Max(int a, int b);

}  // namespace PglMath
