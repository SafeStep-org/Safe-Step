import sys
import logging
import asyncio
import threading
from typing import Any, Union

from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class BLEServer:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.server = None
        self.trigger: Union[asyncio.Event, threading.Event]

        if sys.platform in ["darwin", "win32"]:
            self.trigger = threading.Event()
        else:
            self.trigger = asyncio.Event()

        self.service_uuid = "A07498CA-AD5B-474E-940D-16F1FBE7E8CD"
        self.char_uuid = "51FF12BB-3ED8-46E5-B4F9-D64E2FEC021B"

    def read_request(self, characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
        logger.debug(f"Reading value: {characteristic.value}")
        return characteristic.value

    def write_request(self, characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
        characteristic.value = value
        logger.debug(f"Received write: {value}")
        if value == b"\x0f":
            logger.debug("Trigger value received!")
            self.trigger.set()

    async def start(self):
        self.trigger.clear()
        self.server = BlessServer(name="Safe-Pi", loop=self.loop)
        self.server.read_request_func = self.read_request
        self.server.write_request_func = self.write_request

        # Add service and characteristic
        await self.server.add_new_service(self.service_uuid)

        props = (
            GATTCharacteristicProperties.read
            | GATTCharacteristicProperties.write
            | GATTCharacteristicProperties.indicate
        )
        perms = (
            GATTAttributePermissions.readable
            | GATTAttributePermissions.writeable
        )

        await self.server.add_new_characteristic(
            self.service_uuid,
            self.char_uuid,
            props,
            None,
            perms,
        )

        await self.server.start()
        logger.info("BLE server started and advertising")

        if isinstance(self.trigger, threading.Event):
            self.trigger.wait()
        else:
            await self.trigger.wait()

    async def send(self, data: str):
        if self.server is None:
            logger.warning("BLE server not started.")
            return

        char = self.server.get_characteristic(self.char_uuid)
        if char is None:
            logger.warning("Characteristic not found.")
            return

        encoded = data.encode("utf-8")
        logger.debug(f"Sending: {encoded}")
        self.server.update_value(self.service_uuid, self.char_uuid, encoded)

    async def stop(self):
        if self.server:
            await self.server.stop()
            logger.info("BLE server stopped.")
