import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

void main() {
  runApp(BleApp());
}

class BleApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'BLE Sensor Reader',
      home: SensorScreen(),
    );
  }
}

class SensorScreen extends StatefulWidget {
  @override
  _SensorScreenState createState() => _SensorScreenState();
}

class _SensorScreenState extends State<SensorScreen> {
  final _ble = FlutterReactiveBle();
  final serviceUuid = Uuid.parse("12345678-1234-5678-1234-56789abcdef0");
  final objectUuid = Uuid.parse("12345678-1234-5678-1234-56789abcdef1");
  final distUuid = Uuid.parse("12345678-1234-5678-1234-56789abcdef2");

  late DiscoveredDevice device;
  String object = "N/A";
  String distance = "N/A";

  @override
  void initState() {
    super.initState();
    scanAndConnect();
  }

  void scanAndConnect() {
    _ble.scanForDevices(withServices: [serviceUuid]).listen((d) async {
      if (d.name.isNotEmpty) {
        device = d;
        _ble.connectToDevice(id: device.id).listen((_) async {
          final chars = await _ble.discoverServices(device.id);
          final objChar = chars
              .expand((s) => s.characteristics)
              .firstWhere((c) => c.characteristicId == objectUuid);
          final distChar = chars
              .expand((s) => s.characteristics)
              .firstWhere((c) => c.characteristicId == distUuid);

          _ble.subscribeToCharacteristic(objChar).listen((data) {
            setState(() => object = String.fromCharCodes(data));
          });

          _ble.subscribeToCharacteristic(distChar).listen((data) {
            setState(() => distance = String.fromCharCodes(data));
          });
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text("Sensor Reader")),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text("Detected Object: $object", style: TextStyle(fontSize: 24)),
            SizedBox(height: 20),
            Text("Distance: $distance m", style: TextStyle(fontSize: 24)),
          ],
        ),
      ),
    );
  }
}
