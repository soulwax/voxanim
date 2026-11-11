/// High-performance skeletal animation blending, 1D/2D blend trees,
/// and inverse kinematics (TwoBoneIK, FABRIK, AimIK) for Dart.
library voxanim;

export 'src/math/anim_vec3.dart';
export 'src/math/anim_quat.dart';

export 'src/pose/bone.dart';
export 'src/pose/pose.dart';
export 'src/pose/skeleton.dart';

export 'src/blend/blend_node.dart';
export 'src/blend/clip_node.dart';
export 'src/blend/blend_1d.dart';
export 'src/blend/blend_2d.dart';
export 'src/blend/additive_blend.dart';

export 'src/ik/two_bone_ik.dart';
export 'src/ik/fabrik_ik.dart';
export 'src/ik/aim_ik.dart';
