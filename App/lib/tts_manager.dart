import 'package:flutter_tts/flutter_tts.dart';

class TtsManager {
  static final TtsManager _instance = TtsManager._internal();
  factory TtsManager() => _instance;
  TtsManager._internal();

  final FlutterTts flutterTts = FlutterTts();

  void speak(String text) async {
    flutterTts.speak(text);
  }
}
