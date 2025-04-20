import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';

class Settings extends StatelessWidget {
  Settings({super.key});

  final FlutterTts flutterTts = FlutterTts();

  @override
  Widget build(BuildContext context) {
    return const Center(
      child: Text('Settings'),
    );
  }
}
