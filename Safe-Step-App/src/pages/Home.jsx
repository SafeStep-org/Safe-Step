import { useState, useEffect } from "react";
import NavBar from "../components/NavBar";

function Home() {
  const [btAllowed, setBtAllowed] = useState(false);

  useEffect(() => {
    const checkBluetoothPermission = async () => {
      const btPermission = await navigator.permissions.query({ name: "bluetooth" });
      if (btPermission.state !== "denied") {
        setBtAllowed(true);
      }
    };

    checkBluetoothPermission();
  }, []);

  return (
    <div className="flex flex-col h-screen justify-between">
      <main className="mb-auto mx-2">
        Home
        {btAllowed && <p>Bluetooth is allowed on this device.</p>}
      </main>

      <footer><NavBar /></footer>
    </div>
  );
}

export default Home;