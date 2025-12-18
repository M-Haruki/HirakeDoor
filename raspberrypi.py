# BleTest.py
# https://logikara.blog/picow-ble-micropython を参照して作成
# BLEのPeripheralとして振る舞う

import aioble  # MicroPythonのBLEライブラリ
import bluetooth  # MicroPythonのBLEモジュール
import uasyncio as asyncio  # 非同期処理ライブラリ
from machine import Pin  # PicoのGPIOモジュールをインポート

UART_SERVICE_UUID = bluetooth.UUID(
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
)  # UARTサービスUUID
UART_RX_CHAR_UUID = bluetooth.UUID(
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
)  # UART受信用キャラクタリスティックUUID
UART_TX_CHAR_UUID = bluetooth.UUID(
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
)  # UART送信用キャラクタリスティックUUID

# サービスとキャラクタリスティックの定義
uart_service = aioble.Service(UART_SERVICE_UUID)  # UARTサービスを定義
tx_char = aioble.Characteristic(
    uart_service, UART_TX_CHAR_UUID, read=True, notify=True
)  # UART送信用キャラクタリスティックを定義
rx_char = aioble.Characteristic(
    uart_service, UART_RX_CHAR_UUID, write=True, write_no_response=True, capture=True
)  # UART受信用キャラクタリスティックを定義

aioble.register_services(uart_service)  # GATTサービスを登録

led = Pin("LED", Pin.OUT)  # LED端子をledとして出力に設定
out_a = Pin(16, Pin.OUT)
out_b = Pin(17, Pin.OUT)
out_a.value(0)  # 初期状態はOFF
out_b.value(0)
# sw_main = Pin(14, Pin.IN, Pin.PULL_DOWN)  # 送信ボタン
sw_reset = Pin(15, Pin.IN, Pin.PULL_DOWN)  # 接続解除用ボタン


# 非同期でタスクを実行
async def handle_rx(connection):
    # 受信待ちループ
    while connection.is_connected():
        try:
            _, value = await rx_char.written()  # 戻り値のconnectionは未使用
            print("Received:", value)
            v = value.strip().lower()
            if v == b"front":
                out_a.value(1)
                out_b.value(0)
            elif v == b"back":
                out_a.value(0)
                out_b.value(1)
            elif v == b"stop":
                out_a.value(1)
                out_b.value(1)
            elif v == b"off":
                out_a.value(0)
                out_b.value(0)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print("ReceiveError:", e)
            break


# async def handle_tx(connection):
#     # 送信ループ（変化時のみ通知）
#     last = None
#     while connection.is_connected():
#         try:
#             value = "on" if sw_main.value() else "off"
#             if value != last:
#                 last = value
#                 try:
#                     await tx_char.notify(connection, value.encode())
#                 except asyncio.CancelledError:
#                     raise
#                 except Exception:
#                     # 未購読や一時的な送信失敗はスキップ
#                     pass
#         except asyncio.CancelledError:
#             raise
#         except Exception as e:
#             print("SendError:", e)
#             break
#         await asyncio.sleep_ms(100)


async def handle_disconnect(connection):
    # 接続解除ボタン用ループ
    while connection.is_connected():
        try:
            if sw_reset.value():
                await connection.disconnect()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print("DisconnectError:", e)
            break
        await asyncio.sleep_ms(100)


# BLE Peripheralとして動作するメインループ
async def ble_peripheral_loop():
    led.value(1)
    await asyncio.sleep_ms(3000)  # 起動後3秒間LED点灯
    led.value(0)
    while True:
        led.value(0)  # 接続待機中はLED消灯
        print("Waiting connect...")
        try:
            async with await aioble.advertise(  # BLEアドバタイズを開始
                250_000,  # アドバタイズ間隔（マイクロ秒）
                name="HirakeDoma_RPiPico",  # アドバタイズ名（BLE接続デバイス名）
                services=[UART_SERVICE_UUID],  # サービスUUIDを指定
            ) as connection:  # 接続待機
                print("Connecting:", connection.device)  # 接続されたデバイス情報を表示
                led.value(1)

                # 受信処理を別タスクで開始
                rx_task = asyncio.create_task(handle_rx(connection))
                # tx_task = asyncio.create_task(handle_tx(connection))
                disconnect_task = asyncio.create_task(handle_disconnect(connection))

                # 接続が切れるまで待機
                while connection.is_connected():
                    await asyncio.sleep_ms(100)

                # 接続解除 → タスクキャンセル
                rx_task.cancel()
                # tx_task.cancel()
                disconnect_task.cancel()
                led.value(0)  # LED消灯
                print("Disconnected")

        except Exception as e:
            print("Advertise:", repr(e))

        out_a.value(0)
        out_b.value(0)
        await asyncio.sleep_ms(200)


# 実行
try:
    # PicoのBLE Peripheralとして動作開始
    asyncio.run(ble_peripheral_loop())
except KeyboardInterrupt:
    led.value(0)
    out_a.value(0)
    out_b.value(0)
    print("Cleanup done.")
