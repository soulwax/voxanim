import '../pose/pose.dart';
import 'blend_node.dart';

/// Additive animation blend node layering secondary delta animation on top of a base pose.
class AdditiveBlendNode extends BlendNode {
  final BlendNode baseNode;
  final BlendNode additiveNode;
  final String weightParameter;

  AdditiveBlendNode({
    required this.baseNode,
    required this.additiveNode,
    this.weightParameter = 'weight',
  });

  @override
  Pose evaluate(double time, Map<String, double> parameters) {
    final basePose = baseNode.evaluate(time, parameters).copy();
    final weight = (parameters[weightParameter] ?? 1.0).clamp(0.0, 1.0);

    if (weight > 0.0) {
      final deltaPose = additiveNode.evaluate(time, parameters);
      basePose.applyAdditive(deltaPose, weight);
    }

    return basePose;
  }
}
