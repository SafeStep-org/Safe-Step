import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:open_route_service/open_route_service.dart';

class MapScreen extends StatefulWidget {
  const MapScreen({super.key});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  final defaultPoint = LatLng(39.5073, -84.7452); // Centered on Oxford, OH
  late LatLng myPoint;
  bool isLoading = false;

  List<LatLng> points = [];
  List<Marker> markers = [];

  final MapController mapController = MapController();

  @override
  void initState() {
    myPoint = defaultPoint;
    super.initState();
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
      profile: ORSProfile.footWheelchair, // Accessibility-focused routing
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
            builder: (context) => IconButton(
              onPressed: () {},
              icon: const Icon(Icons.accessibility),
              color: Colors.black,
              iconSize: 45,
            ),
          ),
        );
      }

      if (markers.length == 1) {
        mapController.move(latLng, 17.0);
      }

      if (markers.length == 2) {
        Future.delayed(const Duration(milliseconds: 300), () {
          setState(() {
            isLoading = true;
          });
        });

        getCoordinates(markers[0].point, markers[1].point);

        LatLngBounds bounds = LatLngBounds.fromPoints(
            markers.map((marker) => marker.point).toList());
        mapController.fitBounds(bounds);
      }
    });
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
