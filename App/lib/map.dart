import 'dart:async';
import 'dart:ui';
import 'dart:convert';
import 'package:http/http.dart' as http;

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:open_route_service/open_route_service.dart';
import 'package:geolocator/geolocator.dart';

class Map extends StatefulWidget {
  const Map({super.key});

  @override
  State<Map> createState() => _MapScreenState();
}

class _MapScreenState extends State<Map> {
  final defaultPoint = LatLng(39.5073, -84.7452); // Oxford, OH
  late LatLng myPoint;
  bool isLoading = false;

  List<LatLng> points = [];
  List<Marker> markers = [];

  final MapController mapController = MapController();

  LatLng? currentUserLocation;
  Timer? locationUpdateTimer;

  @override
  void initState() {
    myPoint = defaultPoint;
    super.initState();
    _initializeLocation();
  }

  Future<void> _initializeLocation() async {
    LocationPermission permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied ||
        permission == LocationPermission.deniedForever) {
      permission = await Geolocator.requestPermission();
    }

    final Position position = await Geolocator.getCurrentPosition(
      desiredAccuracy: LocationAccuracy.high,
    );
    if (!mounted) return; // <-- Add this

    currentUserLocation = LatLng(position.latitude, position.longitude);
    myPoint = currentUserLocation!;

    _startLiveLocationUpdates();

    setState(() {
      markers.add(
        Marker(
          point: currentUserLocation!,
          width: 80,
          height: 80,
          child: Icon(Icons.accessibility_new, size: 45, color: Colors.blue),
        ),
      );
    });
  }

  void _startLiveLocationUpdates() {
    locationUpdateTimer = Timer.periodic(const Duration(seconds: 2), (
      Timer t,
    ) async {
      if (!mounted) return;

      Position position = await Geolocator.getCurrentPosition(
        desiredAccuracy: LocationAccuracy.high,
      );
      LatLng newLocation = LatLng(position.latitude, position.longitude);

      if (currentUserLocation != null) {
        double distanceMoved = const Distance().as(
          LengthUnit.Meter,
          currentUserLocation!,
          newLocation,
        );

        if (distanceMoved > 5 && markers.length == 2) {
          currentUserLocation = newLocation;

          markers[0] = Marker(
            point: newLocation,
            width: 80,
            height: 80,
            child: const Icon(
              Icons.accessibility_new,
              size: 45,
              color: Colors.blue,
            ),
          );

          await getCoordinates(newLocation, markers[1].point);

          if (!mounted) return;
          setState(() {});
        }
      }
    });
  }

  // Define this model globally to store instructions
  List<String> navigationInstructions = [];
  
  Future<void> getCoordinates(LatLng start, LatLng end) async {
    if (!mounted) return;
  
    setState(() {
      isLoading = true;
    });
  
    const apiKey = 'YOUR_API_KEY_HERE'; // Replace with your OpenRouteService API key
  
    final url = Uri.parse('https://api.openrouteservice.org/v2/directions/wheelchair/geojson');
  
    final body = jsonEncode({
      "coordinates": [
        [start.longitude, start.latitude], // OpenRouteService expects [lng, lat]!
        [end.longitude, end.latitude]
      ]
    });
  
    final response = await http.post(
      url,
      headers: {
        'Authorization': apiKey,
        'Content-Type': 'application/json',
      },
      body: body,
    );
  
    if (response.statusCode == 200) {
      final data = json.decode(response.body);
  
      // Extract route coordinates
      final coords = data['features'][0]['geometry']['coordinates'] as List;
  
      final List<LatLng> routePoints = coords.map<LatLng>((coord) {
        return LatLng(coord[1], coord[0]); // Reverse [lng, lat] -> [lat, lng]
      }).toList();
  
      // Extract navigation instructions
      final steps = data['features'][0]['properties']['segments'][0]['steps'] as List;
  
      navigationInstructions = steps.map<String>((step) {
        final instruction = step['instruction'];
        final distance = step['distance'];
        return '$instruction in ${distance.toStringAsFixed(0)} meters';
      }).toList();
  
      if (!mounted) return;
  
      setState(() {
        points = routePoints;
        isLoading = false;
      });
  
      // For debugging, you could print instructions
      for (var instr in navigationInstructions) {
        print(instr);
      }
  
    } else {
      if (!mounted) return;
      setState(() {
        isLoading = false;
      });
      throw Exception('Failed to get route: ${response.body}');
    }
  }

  void _handleTap(LatLng latLng) {
    setState(() {
      if (markers.length < 2) {
        markers.add(
          Marker(
            point: latLng,
            width: 80,
            height: 80,
            child: Icon(Icons.flag, color: Colors.red, size: 45),
          ),
        );
      }

      if (markers.length == 2 && currentUserLocation != null) {
        Future.delayed(const Duration(milliseconds: 300), () {
          if (!mounted) return;

          setState(() {
            isLoading = true;
          });
        });

        getCoordinates(currentUserLocation!, markers[1].point);
      }
    });
  }

  @override
  void dispose() {
    locationUpdateTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          FlutterMap(
            mapController: mapController,
            options: MapOptions(
              initialZoom: 16,
              initialCenter: myPoint,
              onTap: (tapPosition, latLng) => _handleTap(latLng),
            ),
            children: [
              TileLayer(
                urlTemplate: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                userAgentPackageName: 'dev.fleaflet.flutter_map.example',
              ),
              MarkerLayer(markers: markers),
              if (points.isNotEmpty)
                PolylineLayer(
                  polylines: [
                    Polyline(
                      points: points,
                      color: Colors.green,
                      strokeWidth: 5,
                    ),
                  ],
                ),
            ],
          ),
          Visibility(
            visible: isLoading,
            child: Container(
              color: Colors.black.withOpacity(0.5),
              child: BackdropFilter(
                filter: ImageFilter.blur(sigmaX: 5, sigmaY: 5),
                child: const Center(
                  child: CircularProgressIndicator(color: Colors.white),
                ),
              ),
            ),
          ),
          Positioned(
            top: MediaQuery.of(context).padding.top + 20.0,
            left: MediaQuery.of(context).size.width / 2 - 110,
            child: TextButton(
              onPressed: () {
                setState(() {
                  markers = [];
                  points = [];

                  if (currentUserLocation != null) {
                    markers.add(
                      Marker(
                        point: currentUserLocation!,
                        width: 80,
                        height: 80,
                        child: Icon(
                          Icons.accessibility_new,
                          size: 45,
                          color: Colors.blue,
                        ),
                      ),
                    );
                  }
                });
              },
              child: Container(
                width: 200,
                height: 50,
                decoration: BoxDecoration(
                  color: Colors.black,
                  borderRadius: BorderRadius.circular(10),
                ),
                child: const Center(
                  child: Text(
                    "Clear route",
                    style: TextStyle(color: Colors.white, fontSize: 18),
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
