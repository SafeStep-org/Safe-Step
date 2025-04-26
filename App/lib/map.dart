import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:geolocator/geolocator.dart';
import 'package:http/http.dart' as http;
import 'package:latlong2/latlong.dart';

class NavigationStep {
  final LatLng location;
  final String instruction;
  bool announced;

  NavigationStep({
    required this.location,
    required this.instruction,
    this.announced = false,
  });
}

class NavigationPage extends StatefulWidget {
  final LatLng start;
  final LatLng end;

  const NavigationPage({super.key, required this.start, required this.end});

  @override
  State<NavigationPage> createState() => _NavigationPageState();
}

class _NavigationPageState extends State<NavigationPage> {
  final mapController = MapController();

  List<NavigationStep> navigationSteps = [];
  List<LatLng> routePoints = [];
  List<Marker> markers = [];

  LatLng? currentUserLocation;
  Timer? locationUpdateTimer;

  @override
  void initState() {
    super.initState();
    _initializeRoute();
  }

  @override
  void dispose() {
    locationUpdateTimer?.cancel();
    super.dispose();
  }

  Future<void> _initializeRoute() async {
    final response = await http.post(
      Uri.parse('https://api.openrouteservice.org/v2/directions/foot-walking/geojson'),
      headers: {
        'Authorization': '5b3ce3597851110001cf6248afaea8a79e6d4f7891520a594e9fbf77',
        'Content-Type': 'application/json',
      },
      body: jsonEncode({
        "coordinates": [
          [widget.start.longitude, widget.start.latitude],
          [widget.end.longitude, widget.end.latitude]
        ],
        "extra_info": ["wheelchair", "surface", "steepness"]
      }),
    );

    if (response.statusCode == 200) {
      final data = jsonDecode(response.body);
      final features = data['features'][0];
      final coords = features['geometry']['coordinates'] as List;

      routePoints = coords
          .map((c) => LatLng(c[1] as double, c[0] as double))
          .toList();

      final steps = features['properties']['segments'][0]['steps'] as List;

      for (var step in steps) {
        final instruction = step['instruction'];
        final waypoints = step['way_points'];
        final pointCoord = coords[waypoints[0]];
        final location = LatLng(pointCoord[1] as double, pointCoord[0] as double);

        navigationSteps.add(NavigationStep(
          location: location,
          instruction: instruction,
        ));
      }

      markers = [
        Marker(
          point: widget.start,
          width: 80,
          height: 80,
          child: const Icon(Icons.location_on, color: Colors.green, size: 45),
        ),
        Marker(
          point: widget.end,
          width: 80,
          height: 80,
          child: const Icon(Icons.flag, color: Colors.red, size: 45),
        ),
      ];

      setState(() {});
      _startLiveLocationUpdates();
    } else {
      print('Failed to fetch route: ${response.body}');
    }
  }

  void _startLiveLocationUpdates() {
    locationUpdateTimer = Timer.periodic(const Duration(seconds: 2), (Timer t) async {
      if (!mounted) return;

      Position position = await Geolocator.getCurrentPosition(
        desiredAccuracy: LocationAccuracy.high,
      );

      LatLng newLocation = LatLng(position.latitude, position.longitude);

      if (currentUserLocation == null || const Distance().as(LengthUnit.Meter, currentUserLocation!, newLocation) > 3) {
        currentUserLocation = newLocation;

        markers[0] = Marker(
          point: newLocation,
          width: 80,
          height: 80,
          child: const Icon(Icons.accessibility_new, color: Colors.blue, size: 45),
        );

        _checkNavigationSteps(newLocation);

        if (!mounted) return;
        setState(() {});
      }
    });
  }

  void _checkNavigationSteps(LatLng currentLocation) {
    for (var step in navigationSteps) {
      if (!step.announced) {
        double distanceToStep = const Distance().as(
          LengthUnit.Meter,
          currentLocation,
          step.location,
        );

        if (distanceToStep < 20) {
          print("NAVIGATION: ${step.instruction} in ${distanceToStep.toStringAsFixed(0)} meters");
          step.announced = true;
          break; // announce only one at a time
        }
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Accessible Navigation")),
      body: FlutterMap(
        mapController: mapController,
        options: MapOptions(
          center: widget.start,
          zoom: 16,
        ),
        children: [
          TileLayer(
            urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
            userAgentPackageName: 'com.example.app',
          ),
          PolylineLayer(
            polylines: [
              Polyline(
                points: routePoints,
                color: Colors.blue,
                strokeWidth: 4,
              ),
            ],
          ),
          MarkerLayer(markers: markers),
        ],
      ),
    );
  }
}
