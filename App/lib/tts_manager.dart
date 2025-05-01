import 'package:flutter_tts/flutter_tts.dart';

class TtsManager {
  static final TtsManager _instance = TtsManager._internal();
  factory TtsManager() => _instance;
  TtsManager._internal();

  final FlutterTts flutterTts = FlutterTts();

  void speak(String text) async {
    flutterTts.speak(text);
  }

  void speakImportant(String text) async {
    await flutterTts.stop();
    print("speaking $text");
    await flutterTts.speak(text);
  }

  void stop() async {
    flutterTts.stop();
  }
}
