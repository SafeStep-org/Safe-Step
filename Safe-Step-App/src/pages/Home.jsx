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
        filters: [{ services: ["302c754d-63c1-4c28-a5ff-ad3e9f332226"] }],
      });

      const server = await device.gatt.connect();
      const service = await server.getPrimaryService(
        "302c754d-63c1-4c28-a5ff-ad3e9f332226"
      );
      const characteristic = await service.getCharacteristic(
        "3a98b215-2971-4c6d-b5c2-02597ae99d0e"
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
