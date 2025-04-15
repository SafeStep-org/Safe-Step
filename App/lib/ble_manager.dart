import 'dart:async';
import 'dart:developer';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class BleManager {
  static final BleManager _instance = BleManager._internal();
  factory BleManager() => _instance;
  BleManager._internal();

  final Guid serviceUuid = Guid("302c754d-63c1-4c28-a5ff-ad3e9f332226");
  final Guid characteristicUuid = Guid("3a98b215-2971-4c6d-b5c2-02597ae99d0e");

  BluetoothDevice? connectedDevice;
  BluetoothCharacteristic? targetCharacteristic;

  final _connectionStatusController = StreamController<bool>.broadcast();
  Stream<bool> get connectionStatusStream => _connectionStatusController.stream;

  Function(String message)? onMessageReceived;

  Future<void> connectToDevice(BluetoothDevice device) async {
    if (connectedDevice?.id == device.id) {
      log("Already connected to ${device.name}");
      return;
    }

    await disconnect();

    try {
      await device.connect(timeout: const Duration(seconds: 10));
      connectedDevice = device;

      List<BluetoothService> services = await device.discoverServices();
      for (var service in services) {
        if (service.uuid == serviceUuid) {
          for (var char in service.characteristics) {
            if (char.uuid == characteristicUuid) {
              targetCharacteristic = char;
              char.setNotifyValue(true);

              _connectionStatusController.add(true);
              await writeData([0x0f]);

              return;
            }
          }
        }
      }
       
      log("Target service/characteristic not found.");
      _connectionStatusController.add(false);
    } catch (e) {
      log("Connection error: $e");
      _connectionStatusController.add(false);
    }
  }

  Future<void> writeData(List<int> data) async {
    if (targetCharacteristic != null) {
      await targetCharacteristic!.write(data);
    } else {
      log("Characteristic not found");
    }
  }

  Future<void> disconnect() async {
    if (connectedDevice != null) {
      await connectedDevice!.disconnect();
      connectedDevice = null;
      targetCharacteristic = null;
      _connectionStatusController.add(false);
    }
  }
}
