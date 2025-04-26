import 'dart:async';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:open_route_service/open_route_service.dart';
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

  String? selectedDestination; // Track selected destination

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

  Future<void> getCoordinates(LatLng lat1, LatLng lat2) async {
    if (!mounted) return;

    setState(() {
      isLoading = true;
    });

    final OpenRouteService client = OpenRouteService(
      apiKey: '5b3ce3597851110001cf6248afaea8a79e6d4f7891520a594e9fbf77',
    );

    final List<ORSCoordinate> routeCoordinates = await client
        .directionsRouteCoordsGet(
          startCoordinate: ORSCoordinate(
            latitude: lat1.latitude,
            longitude: lat1.longitude,
          ),
          endCoordinate: ORSCoordinate(
            latitude: lat2.latitude,
            longitude: lat2.longitude,
          ),
        );

    final List<LatLng> routePoints =
        routeCoordinates
            .map(
              (coordinate) => LatLng(coordinate.latitude, coordinate.longitude),
            )
            .toList();

    if (!mounted) return;

    setState(() {
      points = routePoints;
      isLoading = false;
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
