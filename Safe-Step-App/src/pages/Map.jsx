import NavBar from "../components/NavBar.jsx";
import DirectionsMap from "../components/DirectionsMap.jsx";

function Map() {
  return (
    <div className="flex flex-col h-screen w-screen justify-between">
      <main className="mb-auto w-full h-full">
        <DirectionsMap />
      </main>

      <footer><NavBar /></footer>
    </div>
  );
}

export default Map;