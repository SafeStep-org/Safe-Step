import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:open_route_service/open_route_service.dart';
import 'package:geolocator/geolocator.dart';

class MapScreen extends StatefulWidget {
  const MapScreen({super.key});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
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
    if (permission == LocationPermission.denied || permission == LocationPermission.deniedForever) {
      permission = await Geolocator.requestPermission();
    }

    final Position position = await Geolocator.getCurrentPosition(
      desiredAccuracy: LocationAccuracy.high,
    );
    currentUserLocation = LatLng(position.latitude, position.longitude);
    myPoint = currentUserLocation!;
    _startLiveLocationUpdates();

    setState(() {
      markers.add(
        Marker(
          point: currentUserLocation!,
          width: 80,
          height: 80,
          builder: (context) => const Icon(Icons.accessibility_new, size: 45, color: Colors.blue),
        ),
      );
    });
  }

  void _startLiveLocationUpdates() {
    locationUpdateTimer = Timer.periodic(const Duration(seconds: 2), (Timer t) async {
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
            builder: (context) => const Icon(Icons.accessibility_new, size: 45, color: Colors.blue),
          );

          await getCoordinates(newLocation, markers[1].point);
          setState(() {});
        }
      }
    });
  }

  Future<void> getCoordinates(LatLng lat1, LatLng lat2) async {
    setState(() {
      isLoading = true;
    });

    final OpenRouteService client = OpenRouteService(
      apiKey: '5b3ce3597851110001cf6248afaea8a79e6d4f7891520a594e9fbf77',
    );

    final List<ORSCoordinate> routeCoordinates =
        await client.directionsRouteCoordsGet(
      startCoordinate:
          ORSCoordinate(latitude: lat1.latitude, longitude: lat1.longitude),
      endCoordinate:
          ORSCoordinate(latitude: lat2.latitude, longitude: lat2.longitude),
      profile: ORSProfile.footWheelchair,
    );

    final List<LatLng> routePoints = routeCoordinates
        .map((coordinate) => LatLng(coordinate.latitude, coordinate.longitude))
        .toList();

    setState(() {
      points = routePoints;
      isLoading = false;
    });
  }

  void _handleTap(LatLng latLng) {
    setState(() {
      if (markers.length < 2) {
        markers.add(
          Marker(
            point: latLng,
            width: 80,
            height: 80,
            builder: (context) => const Icon(Icons.flag, color: Colors.red, size: 45),
          ),
        );
      }

      if (markers.length == 2 && currentUserLocation != null) {
        Future.delayed(const Duration(milliseconds: 300), () {
          setState(() {
            isLoading = true;
          });
        });

        getCoordinates(currentUserLocation!, markers[1].point);

        LatLngBounds bounds = LatLngBounds.fromPoints(
            markers.map((marker) => marker.point).toList());
        mapController.fitBounds(bounds);
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
              zoom: 16,
              center: myPoint,
              onTap: (tapPosition, latLng) => _handleTap(latLng),
            ),
            children: [
              TileLayer(
                urlTemplate: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                userAgentPackageName: 'dev.fleaflet.flutter_map.example',
              ),
              MarkerLayer(markers: markers),
              PolylineLayer(
                polylineCulling: false,
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
                  child: CircularProgressIndicator(
                    color: Colors.white,
                  ),
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
                        builder: (context) => const Icon(Icons.accessibility_new, size: 45, color: Colors.blue),
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
                    borderRadius: BorderRadius.circular(10)),
                child: const Center(
                  child: Text(
                    "Limpar rota",
                    style: TextStyle(color: Colors.white, fontSize: 18),
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
      floatingActionButton: Column(
        mainAxisAlignment: MainAxisAlignment.end,
        children: [
          const SizedBox(height: 10),
          FloatingActionButton(
            backgroundColor: Colors.black,
            onPressed: () {
              mapController.move(mapController.center, mapController.zoom + 1);
            },
            child: const Icon(Icons.add, color: Colors.white),
          ),
          const SizedBox(height: 10),
          FloatingActionButton(
            backgroundColor: Colors.black,
            onPressed: () {
              mapController.move(mapController.center, mapController.zoom - 1);
            },
            child: const Icon(Icons.remove, color: Colors.white),
          ),
        ],
      ),
    );
  }
}
