import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from "react-leaflet";
import { useState, useEffect } from "react";

function DirectionsMap() {
  const [userPosition, setUserPosition] = useState([39.5087, -84.7345]);
  const [destination, setDestination] = useState([39.5087, -84.7345]);
  const [route, setRoute] = useState([]);
  const [locationFetched, setLocationFetch] = useState(false);

  useEffect(() => {
    if(!locationFetched) {
      navigator.geolocation.getCurrentPosition(
        (position) => {
          const { latitude, longitude } = position.coords;
          setUserPosition([latitude, longitude]);
          setLocationFetch(true);
        },
        (error) => {
          console.error("Error getting location: ", error);
        }
      );
    }
  }, [locationFetched]);

  useEffect(() => {
    if (userPosition && destination && locationFetched) {
      const routeUrl = `https://router.project-osrm.org/route/v1/driving/${userPosition[1]},${userPosition[0]};${destination[1]},${destination[0]}?steps=true&geometries=geojson`;

      fetch(routeUrl)
        .then((response) => response.json())
        .then((data) => {
          const route = data.routes[0].geometry.coordinates;
          const latLngs = route.map((coord) => [coord[1], coord[0]]);
          setRoute(latLngs);
          console.log("Route: ", latLngs);
        })
        .catch((err) => {
          console.error("Error fetching directions: ", err);
        });
    }
  }, [userPosition, destination, locationFetched]);

  function SetMap() {
    const map = useMap();
    map.setView(userPosition, 13);
    return null;
  }

  return (
    <MapContainer center={userPosition} zoom={13} className="leaflet-container w-full h-full">
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      />

      {/* User Location Marker */}
      <Marker position={userPosition}>
        <Popup>Your Location</Popup>
      </Marker>

      {/* Destination Marker */}
      <Marker position={destination}>
        <Popup>Destination</Popup>
      </Marker>

      {/* Route Line */}
      {route.length > 0 && (
        <Polyline positions={route} color="blue" weight={4} />
      )}

      <SetMap />
    </MapContainer>
  );
}

export default DirectionsMap;
