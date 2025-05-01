import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:safe_step/ble_manager.dart';
import 'package:safe_step/tts_manager.dart';
import 'package:safe_step/wheelchair_manager.dart';

class Settings extends StatefulWidget {
  const Settings({super.key});
  @override
  SettingsState createState() => SettingsState();
}

class SettingsState extends State<Settings> {
  late BleManager _bleManager;
  late WheelchairManager _wcManager;

  @override
  void initState() {
    super.initState();
    _bleManager = Provider.of<BleManager>(context, listen: false);
    _wcManager = Provider.of<WheelchairManager>(context, listen: false);
  }

  void _shutDown() {
    _bleManager.writeData("shutdown".codeUnits);
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text("Accessibility Settings", style: TextStyle(fontSize: 20)),
          CheckboxListTile(
            title: const Text("Wheelchair Routing"),
            value: _wcManager.wheelChairDirections,
            onChanged: (bool? value) {
              setState(() {
                _wcManager.setWheelChairDirections(value);
              });
            },
          ),
          ElevatedButton(
            onPressed: _shutDown,
            child: Text("Shutdown SafeStep"),
          ),
          const SizedBox(height: 20),
        ],
      ),
    );
  }
}
