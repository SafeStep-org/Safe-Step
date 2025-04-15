import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from "react-leaflet";
import { useState, useEffect } from "react";

function haversineDistance(coords1, coords2) {
  const toRad = (value) => (value * Math.PI) / 180;
  const R = 6371e3; // Earth radius in meters

  const [lat1, lon1] = coords1;
  const [lat2, lon2] = coords2;

  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c; // Distance in meters
}

function DirectionsMap() {
  const [userPosition, setUserPosition] = useState([39.5087, -84.7345]);
  const [destination, setDestination] = useState([39.5087, -84.7345]);
  const [route, setRoute] = useState([]);
  const [lastFetchedPosition, setLastFetchedPosition] = useState([39.5087, -84.7345]);
  
  useEffect(() => {
    const fetchUserPosition = () => {
      navigator.geolocation.getCurrentPosition(
        (position) => {
          const { latitude, longitude } = position.coords;
          const newPosition = [latitude, longitude];

          // Only update if the user moves more than 50 meters
          if (haversineDistance(userPosition, newPosition) > 50) {
            setUserPosition(newPosition);
            console.log("User position updated:", newPosition);
          }
        },
        (error) => {
          console.error("Error getting location:", error);
        }
      );
    };

    // Fetch initial position
    fetchUserPosition();

    // Poll every 10 seconds instead of 1 second
    const positionInterval = setInterval(fetchUserPosition, 10000);

    return () => clearInterval(positionInterval);
  }, [userPosition]);

  useEffect(() => {
    // Debounce API calls: Only fetch if user moved >50m and last request was >10 seconds ago
    if (haversineDistance(lastFetchedPosition, userPosition) > 50) {
      const routeUrl = `https://router.project-osrm.org/route/v1/driving/${userPosition[1]},${userPosition[0]};${destination[1]},${destination[0]}?steps=true&geometries=geojson`;

      fetch(routeUrl)
        .then((response) => response.json())
        .then((data) => {
          if (data.routes.length > 0) {
            const latLngs = data.routes[0].geometry.coordinates.map((coord) => [coord[1], coord[0]]);
            setRoute(latLngs);
            setLastFetchedPosition(userPosition); // Update last fetched position
            console.log("Route updated");
          }
        })
        .catch((err) => {
          console.error("Error fetching directions: ", err);
        });
    }
  }, [userPosition, destination]);

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
      {route.length > 0 && <Polyline positions={route} color="blue" weight={4} />}

      <SetMap />
    </MapContainer>
  );
}

export default DirectionsMap;
