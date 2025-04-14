const bleno = require('bleno');

// Define your UUIDs for the service and characteristic
const serviceUUID = 'A07498CA-AD5B-474E-940D-16F1FBE7E8CD';
const characteristicUUID = '51FF12BB-3ED8-46E5-B4F9-D64E2FEC021B';

let characteristic = null;

// Define the characteristic class
class MyCharacteristic extends bleno.Characteristic {
  constructor() {
    super({
      uuid: characteristicUUID,
      properties: ['read', 'write', 'notify'],  // Allow read, write, and notifications
      value: null,  // No initial value
    });
  }

  // Handle reading the characteristic
  onReadRequest(offset, callback) {
    console.log('Read request received');
    callback(this.RESULT_SUCCESS, Buffer.from('Hello from Safe Pi BLE Server'));
  }

  // Handle writing to the characteristic
  onWriteRequest(data, offset, withoutResponse, callback) {
    console.log('Write request received');
    console.log(`Received data: ${data.toString()}`);
    callback(this.RESULT_SUCCESS);
  }

  // Optional: Handle notifications
  onSubscribe(maxValueSize, updateValueCallback) {
    console.log('Client subscribed to notifications');
    this._updateValueCallback = updateValueCallback;
    // Send a notification every 5 seconds
    setInterval(() => {
      if (this._updateValueCallback) {
        this._updateValueCallback(Buffer.from('Notification from BLE server'));
      }
    }, 5000);
  }

  onUnsubscribe() {
    console.log('Client unsubscribed from notifications');
    this._updateValueCallback = null;
  }
}

// Set up the service and start advertising
bleno.on('stateChange', (state) => {
  if (state === 'poweredOn') {
    console.log('Bluetooth is powered on, starting server...');
    bleno.startAdvertising('Safe-Pi', [serviceUUID]);
  } else {
    console.log('Bluetooth is not powered on, stopping server...');
    bleno.stopAdvertising();
  }
});

// Set up the service and characteristics
bleno.on('advertisingStart', (error) => {
  if (error) {
    console.log('Advertising start failed:', error);
    return;
  }

  console.log('Advertising started');
  // Define the service with our characteristic
  const primaryService = new bleno.PrimaryService({
    uuid: serviceUUID,
    characteristics: [
      new MyCharacteristic()
    ],
  });

  // Set the service
  bleno.setServices([primaryService]);
});
