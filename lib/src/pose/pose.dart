import '../math/anim_quat.dart';
import 'bone.dart';

/// A full skeletal animation pose storing bone transforms indexed by bone name.
class Pose {
  final Map<String, BoneTransform> bones;

  Pose([Map<String, BoneTransform>? initialBones]) : bones = initialBones ?? {};

  BoneTransform? getBone(String name) => bones[name];

  void setBone(String name, BoneTransform transform) {
    bones[name] = transform;
  }

  Pose copy() {
    final copyMap = <String, BoneTransform>{};
    for (final entry in bones.entries) {
      copyMap[entry.key] = entry.value.copy();
    }
    return Pose(copyMap);
  }

  /// Linearly interpolates between two poses [a] and [b] with blending factor [t] in [0, 1].
  static Pose lerp(Pose a, Pose b, double t) {
    final result = Pose();
    final allKeys = {...a.bones.keys, ...b.bones.keys};

    for (final key in allKeys) {
      final boneA = a.bones[key] ?? BoneTransform.identity;
      final boneB = b.bones[key] ?? BoneTransform.identity;
      result.bones[key] = boneA.lerp(boneB, t);
    }

    return result;
  }

  /// Layers an additive delta pose onto this pose with weight [weight] in [0, 1].
  void applyAdditive(Pose deltaPose, double weight) {
    if (weight <= 0.0) return;

    for (final entry in deltaPose.bones.entries) {
      final base = bones[entry.key];
      if (base != null) {
        final delta = entry.value;
        // Position delta
        base.position += delta.position * weight;
        // Rotation delta: slerp between identity and delta rotation, then apply
        final deltaRot = AnimQuat.slerp(AnimQuat.identity, delta.rotation, weight);
        base.rotation = (deltaRot * base.rotation).normalized;
        // Scale delta
        base.scale = base.scale.lerp(base.scale + (delta.scale - BoneTransform.identity.scale), weight);
      }
    }
  }
}
