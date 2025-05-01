class WheelchairRoutingModel extends ChangeNotifier {
  bool _enabled = false;

  bool get isEnabled => _enabled;

  void setEnabled(bool value) {
    _enabled = value;
    notifyListeners();
  }
}
