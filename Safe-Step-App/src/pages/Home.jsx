import { useState } from "react";
import NavBar from "../components/NavBar";

function Home() {
  const [btDevice, setBTDevice] = useState(null);
  const [btCharacteristic, setBTCharacteristic] = useState(null);
  const [btConnected, setBTConnected] = useState(false);

  const connectToSafeStep = async () => {
    try {
      console.log("Connecting to Safe Step...");

      const device = await navigator.bluetooth.requestDevice({
        filters: [{ services: ['a07498ca-ad5b-474e-940d-16f1fbe7e8cd'] }]
      });

      setBTConnected(true);
      setBTDevice(device);
      const server = await device.gatt.connect();
      const service = await server.getPrimaryService('a07498ca-ad5b-474e-940d-16f1fbe7e8cd');
      const characteristic = await service.getCharacteristic('51ff12bb-3ed8-46e5-b4f9-d64e2fec021b');
      setBTCharacteristic(characteristic);

      // Start notifications
      await characteristic.startNotifications();
      characteristic.addEventListener('characteristicvaluechanged', handleNotification);

      console.log("Connected and listening for messages.");

    } catch (error) {
      console.error("Connection failed", error);
    }
  };

  const handleNotification = (event) => {
    const value = event.target.value;
    const decoder = new TextDecoder('utf-8');
    const message = decoder.decode(value);
    console.log("Received:", message);

    // Optionally speak it!
    const utterance = new SpeechSynthesisUtterance(message);
    speechSynthesis.speak(utterance);
  };


  const sendMessage = async () => {
    if (btCharacteristic) {
      const message = new Uint8Array([0x0f]); // match what your server expects
      await btCharacteristic.writeValue(message);
      console.log("Sent 0x0F to server.");
    }
  };

  const speak = () => {
    const utterance = new SpeechSynthesisUtterance("Ethan Chambers I want you so bad please make me you baby girl");
    utterance.voice = speechSynthesis.getVoices()[0]; // pick a voice
    speechSynthesis.speak(utterance);
  }

  return (
    <div className="flex flex-col h-screen justify-between">
      <main className="mb-auto mx-2">
        <button type="button" onClick={connectToSafeStep} className="border border-primary rounded-xs">Click here to connect to Safe Step</button>
        {btDevice && btDevice.name}
        <button type="button" onClick={speak} className="border border-primary rounded-xs">Speak</button>
        <button onClick={sendMessage} className="border border-primary rounded-xs">Send 0x0F to Server</button>
      </main>

      <footer><NavBar /></footer>
    </div>
  );
}

export default Home;