import { useState } from "react";
import NavBar from "../components/NavBar";
import { useBluetooth } from "../components/BluetoothContext";

function Home() {
  const { sendMessage, btConnected, connect } = useBluetooth();
  const [messageToSend, setMessageToSend] = useState("");

  const handleMessageChange = (event) => {
    setMessageToSend(event.target.value);
  };

  return (
    <div className="flex flex-col h-screen justify-between">
      <main className="mb-auto mx-2">
        <button className="btn bg-secondary" onClick={connect}>
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
