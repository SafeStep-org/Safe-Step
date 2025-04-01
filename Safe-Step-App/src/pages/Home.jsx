import { useState } from "react";
import NavBar from "../components/NavBar";

function Home() {
  const [btDevice, setBTDevice] = useState({});
  // A07498CA-AD5B474E-940D-16F1FBE7E8CD

  const connectToSafeStep = () => {
    console.log("Connecting to Safe Step...");

    navigator.bluetooth.requestDevice({
      filters: [{
      services: ['a07498ca-ad5b-474e-940d-16f1fbe7e8cd']
      }]
    })
    .then(device => { setBTDevice(device); console.log(device); setBTConnected(true); return device.gatt.connect(); })
    .catch(error => { console.error(error); });
  }

  return (
    <div className="flex flex-col h-screen justify-between">
      <main className="mb-auto mx-2">
        <button type="button" onClick={connectToSafeStep} className="border border-primary rounded-xs">Click here to connect to Safe Step</button>
        {btDevice && btDevice.name}
      </main>

      <footer><NavBar /></footer>
    </div>
  );
}

export default Home;