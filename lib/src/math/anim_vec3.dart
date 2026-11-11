import 'dart:math' as math;

/// 3D vector for animation, skeletal poses, and IK solvers.
class AnimVec3 {
  final double x;
  final double y;
  final double z;

  const AnimVec3(this.x, this.y, this.z);

  static const AnimVec3 zero = AnimVec3(0.0, 0.0, 0.0);
  static const AnimVec3 one = AnimVec3(1.0, 1.0, 1.0);
  static const AnimVec3 up = AnimVec3(0.0, 1.0, 0.0);
  static const AnimVec3 forward = AnimVec3(0.0, 0.0, 1.0);
  static const AnimVec3 right = AnimVec3(1.0, 0.0, 0.0);

  AnimVec3 operator +(AnimVec3 other) => AnimVec3(x + other.x, y + other.y, z + other.z);
  AnimVec3 operator -(AnimVec3 other) => AnimVec3(x - other.x, y - other.y, z - other.z);
  AnimVec3 operator *(double scalar) => AnimVec3(x * scalar, y * scalar, z * scalar);
  AnimVec3 operator /(double scalar) => AnimVec3(x / scalar, y / scalar, z / scalar);
  AnimVec3 operator -() => AnimVec3(-x, -y, -z);

  double dot(AnimVec3 other) => x * other.x + y * other.y + z * other.z;

  AnimVec3 cross(AnimVec3 other) => AnimVec3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x,
      );

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  AnimVec3 get normalized {
    final len = length;
    if (len < 1e-12) return AnimVec3.zero;
    return AnimVec3(x / len, y / len, z / len);
  }

  double distance(AnimVec3 other) => (this - other).length;
  double distanceSquared(AnimVec3 other) => (this - other).lengthSquared;

  AnimVec3 lerp(AnimVec3 other, double t) => AnimVec3(
        x + (other.x - x) * t,
        y + (other.y - y) * t,
        z + (other.z - z) * t,
      );

  static AnimVec3 slerp(AnimVec3 start, AnimVec3 end, double percent) {
    var dot = start.dot(end).clamp(-1.0, 1.0);
    final theta = math.acos(dot) * percent;
    final relativeVec = (end - start * dot).normalized;
    return start * math.cos(theta) + relativeVec * math.sin(theta);
  }

  @override
  bool operator ==(Object other) =>
      identical(this, other) ||
      other is AnimVec3 &&
          (x - other.x).abs() < 1e-9 &&
          (y - other.y).abs() < 1e-9 &&
          (z - other.z).abs() < 1e-9;

  @override
  int get hashCode => Object.hash(x, y, z);

  @override
  String toString() => 'AnimVec3(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
}
