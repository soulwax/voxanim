import 'dart:math' as math;
import '../pose/pose.dart';
import 'blend_node.dart';

class Blend2DEntry {
  final double posX;
  final double posY;
  final BlendNode node;

  const Blend2DEntry({
    required this.posX,
    required this.posY,
    required this.node,
  });
}

/// 2D Cartesian / Directional Blend Tree interpolating across a 2D parameter space.
class Blend2DNode extends BlendNode {
  final String parameterX;
  final String parameterY;
  final List<Blend2DEntry> entries;

  Blend2DNode({
    required this.parameterX,
    required this.parameterY,
    required this.entries,
  });

  @override
  Pose evaluate(double time, Map<String, double> parameters) {
    if (entries.isEmpty) return Pose();
    if (entries.length == 1) return entries.first.node.evaluate(time, parameters);

    final x = parameters[parameterX] ?? 0.0;
    final y = parameters[parameterY] ?? 0.0;

    // Check for exact hit
    for (final e in entries) {
      final dx = x - e.posX;
      final dy = y - e.posY;
      if (dx * dx + dy * dy < 1e-6) {
        return e.node.evaluate(time, parameters);
      }
    }

    // Inverse distance weighting (Shepard's interpolation)
    final weights = <double>[];
    var totalWeight = 0.0;

    for (final e in entries) {
      final dx = x - e.posX;
      final dy = y - e.posY;
      final dist = math.sqrt(dx * dx + dy * dy);
      final w = 1.0 / (dist * dist);
      weights.add(w);
      totalWeight += w;
    }

    if (totalWeight <= 0.0) {
      return entries.first.node.evaluate(time, parameters);
    }

    // Multi-way blend
    final evaluatedPoses = entries.map((e) => e.node.evaluate(time, parameters)).toList();
    var blendedPose = evaluatedPoses.first;
    var accumulatedWeight = weights.first / totalWeight;

    for (var i = 1; i < evaluatedPoses.length; i++) {
      final normWeight = weights[i] / totalWeight;
      final t = normWeight / (accumulatedWeight + normWeight);
      blendedPose = Pose.lerp(blendedPose, evaluatedPoses[i], t);
      accumulatedWeight += normWeight;
    }

    return blendedPose;
  }
}
