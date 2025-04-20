
import subprocess
import sys
import asyncio
import threading
import logging
from typing import Union, Optional
from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class SafePiBLEServer:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.trigger: Union[asyncio.Event, threading.Event]
        self.trigger = threading.Event() if sys.platform in ["darwin", "win32"] else asyncio.Event()

        self.server = BlessServer(name="Safe-Step", loop=loop)
        self.server.read_request_func = self.read_request
        self.server.write_request_func = self.write_request

        self.service_uuid = "302c754d-63c1-4c28-a5ff-ad3e9f332226"
        self.char_uuid = "3A98B215-2971-4C6D-B5C2-02597AE99D0E"
        self.characteristic: Optional[BlessGATTCharacteristic] = None

    async def start(self):
        await self.server.add_new_service(self.service_uuid)

        char_flags = (
            GATTCharacteristicProperties.read
            | GATTCharacteristicProperties.write
            | GATTCharacteristicProperties.indicate
            | GATTCharacteristicProperties.write_without_response
            | GATTCharacteristicProperties.notify
        )
        permissions = (
            GATTAttributePermissions.readable
            | GATTAttributePermissions.writeable
        )
        await self.server.add_new_characteristic(
            self.service_uuid, self.char_uuid, char_flags, None, permissions
        )

        self.characteristic = self.server.get_characteristic(self.char_uuid)
        await self.server.start()
        logger.info("BLE server started and advertising.")

    async def stop(self):
        await self.server.stop()
        logger.info("BLE server stopped.")

    def read_request(self, characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
        logger.debug(f"Reading {characteristic.value}")
        return characteristic.value

    def write_request(self, characteristic: BlessGATTCharacteristic, value: bytearray, **kwargs):
        message = value.decode('utf-8')
        logger.info(f"Received from client: {message}")
        
        if(message == 'shutdown'):
            subprocess.run(["shutdown", "now"])
        
        self.characteristic.value = value
        self.server.update_value(self.service_uuid, self.char_uuid)
        
        if self.trigger .__module__ == "threading":
            self.trigger.set()
        else:
            self.loop.call_soon_threadsafe(self.trigger.set)

    async def send_message(self, msg: str):
        if self.characteristic:
            self.characteristic.value = msg.encode('utf-8')
            self.server.update_value(self.service_uuid, self.char_uuid)
            logger.info(f"Sent to client: {msg}")
        else:
            logger.warning("No client connected; message not sent.")
