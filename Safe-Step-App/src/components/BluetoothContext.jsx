import { createContext, useContext, useState, useEffect } from "react";

const BluetoothContext = createContext();

export function useBluetooth() {
  return useContext(BluetoothContext);
}

export function BluetoothProvider({ children }) {
  const [device, setDevice] = useState(null);
  const [server, setServer] = useState(null);
  const [btConnected, setBTConnected] = useState(false);
  const [btCharacteristic, setBTCharacteristic] = useState(null);

  const connect = async () => {
    try {
      const chosenDevice = await navigator.bluetooth.requestDevice({
        filters: [{ services: ["a07498ca-ad5b-474e-940d-16f1fbe7e8cd"] }],
      });

      const server = await device.gatt.connect();
      const service = await server.getPrimaryService(
        "a07498ca-ad5b-474e-940d-16f1fbe7e8cd"
      );
      const characteristic = await service.getCharacteristic(
        "51ff12bb-3ed8-46e5-b4f9-d64e2fec021b"
      );
      console.log(`service: ${service}, characteristic ${characteristic}`);
      setBTCharacteristic(characteristic);

      await characteristic.startNotifications();
      characteristic.addEventListener(
        "characteristicvaluechanged",
        handleNotification
      );

      // Create connection to server
      await btCharacteristic.writeValueWithoutResponse(new Uint8Array([0x0f]));
      console.log("Connected and listening for messages.");
      setBTConnected(true);

      chosenDevice.addEventListener("gattserverdisconnected", onDisconnected);
    } catch (err) {
      console.error("Bluetooth connection failed", err);
    }
  };

  const handleNotification = (event) => {
    const value = event.target.value;
    const decoder = new TextDecoder("utf-8");
    const messagePlainText = decoder.decode(value);

    console.log(`recieved ${messagePlainText}`);

    if (messagePlainText.length > 0) {
      try {
        const messageJSON = JSON.parse(messagePlainText);
        handleJSON(messageJSON);
      } catch {
        console.error(`received not json message: ${messagePlainText}`);
      }
    }
  };

  function handleJSON(message) {
    console.log(message);
    if (message["speak"]) {
      speak(message["speak"]);
    } else if (message["message"]) {
      console.log(message["message"]);
    }
  }

  function speak(message) {
    const utterance = new SpeechSynthesisUtterance(message);
    utterance.voice = speechSynthesis.getVoices()[0];
    speechSynthesis.speak(utterance);
  }

  const sendMessage = async (message) => {
    console.log(`Sending ${message} to server...`);
    await btCharacteristic.writeValueWithoutResponse(new Uint8Array(message));
    console.log(`Sent ${message} to server!`);
  };

  const disconnect = () => {
    if (device && device.gatt.connected) {
      device.gatt.disconnect();
    }
  };

  const onDisconnected = () => {
    setIsConnected(false);
    setDevice(null);
    setServer(null);
  };

  useEffect(() => {
    return () => {
      // Clean up on unmount
      if (device) {
        device.removeEventListener("gattserverdisconnected", onDisconnected);
        if (device.gatt.connected) {
          device.gatt.disconnect();
        }
      }
    };
  }, [device]);

  return (
    <BluetoothContext.Provider
      value={{ device, server, btConnected, connect, disconnect, sendMessage }}
    >
      {children}
    </BluetoothContext.Provider>
  );
}
