// import 'package:location/location.dart';

// class LocationService {
//   final Location _location = Location();

//   Future<LocationData> getCurrentLocation() async {
//     bool serviceEnabled = await _location.serviceEnabled();
//     if (!serviceEnabled) {
//       serviceEnabled = await _location.requestService();
//       if (!serviceEnabled) {
//         throw Exception('Location services are disabled.');
//       }
//     }

//     PermissionStatus permissionGranted = await _location.hasPermission();
//     if (permissionGranted == PermissionStatus.denied) {
//       permissionGranted = await _location.requestPermission();
//       if (permissionGranted != PermissionStatus.granted) {
//         throw Exception('Location permissions are denied.');
//       }
//     }

//     return await _location.getLocation();
//   }

//   Stream<LocationData> get locationStream => _location.onLocationChanged;
// }
