import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:latlong2/latlong.dart';

class RoutingService {
  final String _baseUrl = 'http://your-server-ip:8989/route';

  Future<List<LatLng>> getRoute(LatLng start, LatLng end) async {
    final response = await http.get(Uri.parse(
        '$_baseUrl?point=${start.latitude},${start.longitude}&point=${end.latitude},${end.longitude}&profile=wheelchair&locale=en&points_encoded=false'));

    if (response.statusCode == 200) {
      final data = jsonDecode(response.body);
      final points = data['paths'][0]['points']['coordinates'] as List;
      return points
          .map((point) => LatLng(point[1], point[0]))
          .toList();
    } else {
      throw Exception('Failed to load route');
    }
  }
}
