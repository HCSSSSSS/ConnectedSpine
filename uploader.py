import asyncio, requests
from bleak import BleakScanner, BleakClient

CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
FIREBASE_URL = "https://connected-spine-default-rtdb.europe-west1.firebasedatabase.app/events.json"

NODE_NAMES = {0: "Chest", 1: "Head", 2: "WaistL", 3: "WaistR"}

def on_notify(_, data: bytearray):
    try:
        node_id, code, axis, dP, dR = data.decode().strip().split(",")
        evt = {
            "n": int(node_id),
            "name": NODE_NAMES.get(int(node_id), f"Node{node_id}"),
            "e": "BAD" if code == "B" else "RECOVER",
            "a": axis,
            "dP": float(dP),
            "dR": float(dR),
            "createdAt": {".sv": "timestamp"},
        }
    except Exception as e:
        print("bad packet:", data, e); return
    r = requests.post(FIREBASE_URL, json=evt, timeout=5)
    print("uploaded" if r.ok else f"FAIL {r.status_code}", evt)

async def main():
    while True:
        try:
            dev = await BleakScanner.find_device_by_name("ConnectedSpine-S3", timeout=15)
            if not dev:
                print("S3 not found, retrying...")
                await asyncio.sleep(3)
                continue
            disconnected = asyncio.Event()
            async with BleakClient(dev, disconnected_callback=lambda c: disconnected.set()) as client:
                await client.start_notify(CHAR_UUID, on_notify)
                print("connected, listening...")
                await disconnected.wait()          # 断开才返回
                print("disconnected, reconnecting...")
        except Exception as e:
            print("error:", e, "— retrying in 3s")
            await asyncio.sleep(3)

asyncio.run(main())