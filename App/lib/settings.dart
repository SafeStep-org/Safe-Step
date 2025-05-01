import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:safe_step/ble_manager.dart';
import 'package:safe_step/tts_manager.dart';

class Settings extends StatefulWidget {
  const Settings({super.key});
  @override
  SettingsState createState() => SettingsState();
}

class SettingsState extends State<Settings> {
  late BleManager _bleManager;
  late TtsManager _ttsManager;

  bool _wheelchairRouting = false;

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
        const SizedBox(height: 20),
        Text("Accessibility Settings", style: Theme.of(context).textTheme.titleMedium),
        CheckboxListTile(
          title: const Text("Wheelchair Routing"),
          value: _wheelchairRouting,
          onChanged: (bool? value) {
            setState(() {
              _wheelchairRouting = value ?? false;
            });
            // Save to Provider or local state
            context.read<WheelchairRoutingModel>().setEnabled(_wheelchairRouting);
          },
        ),
      ],
    );
  }
}
