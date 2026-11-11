import '../pose/pose.dart';
import 'blend_node.dart';

class Blend1DEntry {
  final double threshold;
  final BlendNode node;

  const Blend1DEntry({
    required this.threshold,
    required this.node,
  });
}

/// 1D Blend Tree interpolating between child nodes along a 1D scalar parameter.
class Blend1DNode extends BlendNode {
  final String parameterName;
  final List<Blend1DEntry> entries;

  Blend1DNode({
    required this.parameterName,
    required List<Blend1DEntry> entries,
  }) : entries = List.of(entries)..sort((a, b) => a.threshold.compareTo(b.threshold));

  @override
  Pose evaluate(double time, Map<String, double> parameters) {
    if (entries.isEmpty) return Pose();
    if (entries.length == 1) return entries.first.node.evaluate(time, parameters);

    final paramVal = parameters[parameterName] ?? entries.first.threshold;

    if (paramVal <= entries.first.threshold) {
      return entries.first.node.evaluate(time, parameters);
    }
    if (paramVal >= entries.last.threshold) {
      return entries.last.node.evaluate(time, parameters);
    }

    for (var i = 0; i < entries.length - 1; i++) {
      final e0 = entries[i];
      final e1 = entries[i + 1];

      if (e0.threshold <= paramVal && paramVal <= e1.threshold) {
        final span = e1.threshold - e0.threshold;
        final alpha = span > 0.0 ? (paramVal - e0.threshold) / span : 0.0;

        final pose0 = e0.node.evaluate(time, parameters);
        final pose1 = e1.node.evaluate(time, parameters);

        return Pose.lerp(pose0, pose1, alpha);
      }
    }

    return entries.last.node.evaluate(time, parameters);
  }
}
