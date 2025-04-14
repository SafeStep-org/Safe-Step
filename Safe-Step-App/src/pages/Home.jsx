import { useState } from "react";
import NavBar from "../components/NavBar";

function Home() {
  const [btCharacteristic, setBTCharacteristic] = useState(null);
  const [messageToSend, setMessageToSend] = useState("");
  const [btConnected, setBTConnected] = useState(false);

  const connectToSafeStep = async () => {
    try {
      console.log("Connecting to Safe Step...");

      const device = await navigator.bluetooth.requestDevice({
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

      // Start notifications
      await characteristic.startNotifications();
      characteristic.addEventListener(
        "characteristicvaluechanged",
        handleNotification
      );

      console.log("Connected and listening for messages.");
      await sendMessage([0x0f]);
      setBTConnected(true);
    } catch (error) {
      console.error("Connection failed", error);
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

  async function sendMessage() {
    if (btCharacteristic) {
      console.log(`Sending ${messageToSend} to server`);
      const messageArr = new Uint8Array(messageToSend);
      await btCharacteristic.writeValue(messageArr);
      console.log(`Sent ${messageToSend} to server.`);
    }
  }

  const handleMessageChange = (event) => {
    setMessageToSend(event.target.value);
  };

  return (
    <div className="flex flex-col h-screen justify-between">
      <main className="mb-auto mx-2">
        <button onClick={connectToSafeStep} className="btn bg-secondary">
          Click here to connect to Safe Step
        </button>
        {btConnected && (
          <div>
            <input
              type="text"
              onChange={handleMessageChange}
              placeholder="Type message for server here"
              className="input"
            />
            <button onClick={sendMessage} className="btn bg-secondary">
              Send!
            </button>
          </div>
        )}
      </main>

      <footer>
        <NavBar />
      </footer>
    </div>
  );
}

export default Home;
