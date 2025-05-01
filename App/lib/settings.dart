import 'package:flutter/material.dart';

import 'package:safe_step/ble_manager.dart';
import 'package:provider/provider.dart';
import 'package:safe_step/tts_manager.dart';

class Settings extends StatefulWidget {
  const Settings({super.key});
  @override
  SettingsState createState() => SettingsState();
}

class SettingsState extends State<Settings> {
  late BleManager _bleManager;
  late TtsManager _ttsManager;

  @override
  void initState() {
    super.initState();
    _bleManager = Provider.of<BleManager>(context, listen: false);
    _ttsManager = Provider.of<TtsManager>(context, listen: false);
  }

  void _shutDown() {
    _bleManager.writeData("shutdown".codeUnits);
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        ElevatedButton(onPressed: _shutDown, child: Text("Shutdown SafeStep")),
      ],
    );
  }
}
