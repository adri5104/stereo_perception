#!/usr/bin/env python3

import asyncio
import websockets
import json
import uuid
import time

async def send_server_info():
    uri = "ws://localhost:8765"
    for i in range(10):
        try:
            async with websockets.connect(uri, subprotocols=["foxglove.websocket.v1"]) as websocket:
                msg = {
                    "op": "serverInfo",
                    "sessionId": str(uuid.uuid4()),
                    "name": "launch_embedded",
                    "capabilities": [],
                }
                await websocket.send(json.dumps(msg))
                print("✅ Sent serverInfo with new sessionId.")
                return
        except Exception as e:
            print(f"❌ Retry {i+1}/10: {e}")
            time.sleep(1)

if __name__ == "__main__":
    asyncio.run(send_server_info())
