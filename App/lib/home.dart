import 'dart:developer';

import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

import 'ble_manager.dart';

class Home extends StatefulWidget {
  @override
  HomeState createState() => HomeState();
}

class HomeState extends State<Home> {
  final BleManager _bleManager = BleManager();
  final Guid targetServiceUuid = BleManager().serviceUuid;
  final List<ScanResult> _scanResults = [];
  final List<String> _log = [];
  final TextEditingController _inputController = TextEditingController();

  bool _isConnected = false;
  bool _scanning = false;

  @override
  void initState() {
    super.initState();
    _requestPermissions();
    _bleManager.connectionStatusStream.listen((connected) {
      setState(() {
        _isConnected = connected;
      });
    });
  }

  Future<void> _requestPermissions() async {
    await Permission.bluetoothScan.request();
    await Permission.bluetoothConnect.request();
    await Permission.location.request();
    _startScan();
  }

  void _startScan() {
    setState(() {
      _scanning = true;
    });

    FlutterBluePlus.startScan(timeout: const Duration(seconds: 10));
    FlutterBluePlus.scanResults.listen((results) {
      for (var result in results) {
        if (!_scanResults.any((r) => r.device.id == result.device.id) &&
            result.advertisementData.serviceUuids.contains(targetServiceUuid)) {
          setState(() {
            _scanResults.add(result);
          });
        }
      }
    }).onDone(() {
      setState(() {
        _scanning = false;
      });
    });
  }

  void _connectToDevice(BluetoothDevice device) async {
    log("Attempting connection to ${device.name}");
    await _bleManager.connectToDevice(device);

    _bleManager.targetCharacteristic?.value.listen((value) {
      final message = String.fromCharCodes(value);
      log("Received: $message");
      setState(() {
        _log.add("Received: $message");
      });
    });
  }

  void _sendMessage() async {
    final text = _inputController.text;
    if (text.isEmpty) return;

    final data = text.codeUnits;
    await _bleManager.writeData(data);

    setState(() {
      _log.add("Sent: $text");
      _inputController.clear();
    });
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16.0),
      child: _isConnected ? _buildConnectedView() : _buildConnectionPrompt(),
    );
  }

  Widget _buildConnectionPrompt() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text("Connect to a BLE device", style: TextStyle(fontSize: 20)),
        const SizedBox(height: 12),
        _scanning ? const Text("Scanning...") : const SizedBox.shrink(),
        Expanded(
          child: ListView.builder(
            itemCount: _scanResults.length,
            itemBuilder: (context, index) {
              final device = _scanResults[index].device;
              return ListTile(
                title: Text(device.name.isEmpty ? "(unknown)" : device.name),
                subtitle: Text(device.id.id),
                trailing: ElevatedButton(
                  onPressed: () => _connectToDevice(device),
                  child: const Text("Connect"),
                ),
              );
            },
          ),
        ),
      ],
    );
  }

  Widget _buildConnectedView() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text("Connected to device", style: TextStyle(fontSize: 20)),
        const SizedBox(height: 20),
        Row(
          children: [
            Expanded(
              child: TextField(
                controller: _inputController,
                decoration: const InputDecoration(
                  labelText: "Send a message",
                  border: OutlineInputBorder(),
                ),
              ),
            ),
            const SizedBox(width: 8),
            ElevatedButton(
              onPressed: _sendMessage,
              child: const Text("Send"),
            ),
          ],
        ),
        const SizedBox(height: 20),
        const Text("Debug Log", style: TextStyle(fontSize: 18)),
        const SizedBox(height: 8),
        Expanded(
          child: Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: Colors.grey.shade200,
              border: Border.all(color: Colors.grey),
              borderRadius: BorderRadius.circular(8),
            ),
            child: ListView.builder(
              itemCount: _log.length,
              itemBuilder: (context, index) => Text(_log[index]),
            ),
          ),
        ),
      ],
    );
  }
}
