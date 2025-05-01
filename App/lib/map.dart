import 'dart:async';
import 'dart:ui';
import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:geolocator/geolocator.dart';

import 'tts_manager.dart';

class MapScreen extends StatefulWidget {
  const MapScreen({super.key});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  late TtsManager _ttsManager;
  final Map<String, LatLng> presetDestinations = {
    "King Library": LatLng(39.5088, -84.7380),
    "McVey Data Science Building": LatLng(39.5113889, -84.7338886),
    "Western Dining Hall": LatLng(39.504586012616755, -84.72809886040675),
  };

  String? selectedDestination;

  final defaultPoint = LatLng(39.5073, -84.7452); // Oxford, OH
  late LatLng myPoint;
  bool isLoading = false;

  List<LatLng> points = [];
  List<Marker> markers = [];

  final MapController mapController = MapController();

  LatLng? currentUserLocation;
  Timer? locationUpdateTimer;

  List<dynamic> steps = [];
  String lastDirection = "";

  @override
  void initState() {
    myPoint = defaultPoint;
    _ttsManager = TtsManager();

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
    if (!mounted) return;

    currentUserLocation = LatLng(position.latitude, position.longitude);
    myPoint = currentUserLocation!;

    _startLiveLocationUpdates();

    setState(() {
      markers.add(
        Marker(
          point: currentUserLocation!,
          width: 80,
          height: 80,
          child: const Icon(
            Icons.accessibility_new,
            size: 45,
            color: Colors.blue,
          ),
        ),
      );
    });

    mapController.move(currentUserLocation!, 16);
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

        if (distanceMoved > 1 && markers.length == 2) {
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

      // Check if the user is near a turn step
      if (steps.isNotEmpty && points.isNotEmpty) {
        for (var step in steps) {
          final stepLocation = LatLng(
            points[step['way_points'][0]].latitude,
            points[step['way_points'][0]].longitude,
          );
          double distanceToTurn = const Distance().as(
            LengthUnit.Meter,
            newLocation,
            stepLocation,
          );

          if (distanceToTurn <= 10) {
            String instruction = step["instruction"];
            String distance = step["distance"].toString();

            String direction = "$instruction $distance meters";

            if (direction != lastDirection) {
              _ttsManager.speakImportant(direction);
            }

            lastDirection = direction;
          }
        }
      }
    });
  }

  List<String> navigationInstructions = [];

  Future<void> getCoordinates(LatLng start, LatLng end) async {
    if (!mounted) return;

    setState(() {
      isLoading = true;
    });

    const apiKey = '5b3ce3597851110001cf6248afaea8a79e6d4f7891520a594e9fbf77';

    final useWheelchair = Provider.of<WheelchairRoutingModel>(context, listen: false).isEnabled;

    final profile = useWheelchair ? 'wheelchair' : 'foot-walking';

    final url = Uri.parse(
      'https://api.openrouteservice.org/v2/directions/$profile/geojson',
    );

    final body = jsonEncode({
      "coordinates": [
        [start.longitude, start.latitude],
        [end.longitude, end.latitude],
      ],
    });

    final response = await http.post(
      url,
      headers: {'Authorization': apiKey, 'Content-Type': 'application/json'},
      body: body,
    );

    if (response.statusCode == 200) {
      final data = json.decode(response.body);

      final coords = data['features'][0]['geometry']['coordinates'] as List;

      List<LatLng> routePoints = [];
      if (points.isEmpty) {
        routePoints =
            coords.map<LatLng>((coord) {
              return LatLng(coord[1], coord[0]);
            }).toList();
      }

      final stepsFromResponse =
          data['features'][0]['properties']['segments'][0]['steps'] as List;
      steps = stepsFromResponse;

      navigationInstructions =
          stepsFromResponse.map<String>((step) {
            final instruction = step['instruction'];
            final distance = step['distance'];
            return '$instruction in ${distance.toStringAsFixed(0)} meters';
          }).toList();

      if (!mounted) return;

      setState(() {
        if(routePoints.isNotEmpty) {
          points = routePoints;
        }
        
        isLoading = false;
      });

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
            options: MapOptions(initialZoom: 16, initialCenter: myPoint),
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
            left: 20,
            right: 20,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 10),
              decoration: BoxDecoration(
                color: Colors.white,
                borderRadius: BorderRadius.circular(10),
                boxShadow: [
                  BoxShadow(
                    color: Colors.black26,
                    blurRadius: 5,
                    offset: Offset(0, 2),
                  ),
                ],
              ),
              child: DropdownButton<String>(
                isExpanded: true,
                hint: const Text("Select a destination"),
                value: selectedDestination,
                underline: SizedBox(),
                items: [
                  const DropdownMenuItem<String>(
                    value: null,
                    child: Text("No destination"),
                  ),
                  ...presetDestinations.keys.map((String destinationName) {
                    return DropdownMenuItem<String>(
                      value: destinationName,
                      child: Text(destinationName),
                    );
                  }),
                ],
                onChanged: (String? newValue) {
                  if (currentUserLocation != null) {
                    setState(() {
                      selectedDestination = newValue;

                      if (newValue == null) {
                        // Clear route and keep only user's marker
                        markers = [
                          Marker(
                            point: currentUserLocation!,
                            width: 80,
                            height: 80,
                            child: const Icon(
                              Icons.accessibility_new,
                              size: 45,
                              color: Colors.blue,
                            ),
                          ),
                        ];
                        points = [];
                      } else {
                        // Set user's marker and destination marker
                        markers = [
                          Marker(
                            point: currentUserLocation!,
                            width: 80,
                            height: 80,
                            child: const Icon(
                              Icons.accessibility_new,
                              size: 45,
                              color: Colors.blue,
                            ),
                          ),
                          Marker(
                            point: presetDestinations[newValue]!,
                            width: 80,
                            height: 80,
                            child: const Icon(
                              Icons.flag,
                              size: 45,
                              color: Colors.red,
                            ),
                          ),
                        ];
                        points = [];
                        isLoading = true;

                        getCoordinates(
                          currentUserLocation!,
                          presetDestinations[newValue]!,
                        );
                      }
                    });
                  }
                },
              ),
            ),
          ),
        ],
      ),
    );
  }
}
