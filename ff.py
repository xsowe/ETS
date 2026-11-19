import cv2
import numpy as np
import sys
import time
import serial

# ============================================================
#  НАСТРОЙКИ
# ============================================================
CAMERA_INDEX = 0
CAM_WIDTH = 1280
CAM_HEIGHT = 720

COM_PORT = "COM25"          # ← замени на свой
BAUD = 115200
ACK_TIMEOUT = 0.5          # сколько ждать ответа от Pico, сек
MAX_RETRIES = 3            # сколько раз повторить при ошибке

COLS = 9
ROWS = 6
CELL_MM = 28.0
PX_PER_MM = 4.0

FIELD_W_PX = int(CELL_MM * COLS * PX_PER_MM)
FIELD_H_PX = int(CELL_MM * ROWS * PX_PER_MM)

DST_QUAD = np.array([
    [0, 0],
    [FIELD_W_PX - 1, 0],
    [FIELD_W_PX - 1, FIELD_H_PX - 1],
    [0, FIELD_H_PX - 1],
], dtype=np.float32)

SAMPLE_R = 4

# ============================================================
#  UART
# ============================================================
def open_uart(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.dtr = True
        ser.rts = True
        time.sleep(2)              # Pico делает delay(3000) в setup
        # читаем "R" (READY)
        deadline = time.time() + 5
        while time.time() < deadline:
            line = ser.readline().decode(errors='ignore').strip()
            if line == "R":
                print("Pico готова (R)")
                return ser
        print("Pico не прислала READY — продолжаем на свой риск")
        return ser
    except Exception as e:
        print(f"[UART] не открылся ({port}): {e}")
        return None


def send_packet(ser, colors, timeout=0.5, retries=3):
    payload = bytearray()
    payload.append(ord('C'))
    payload.extend(colors.tobytes())
    payload.append(ord('\n'))

    for attempt in range(1, retries + 1):
        ser.reset_input_buffer()
        ser.write(payload)
        ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = ser.readline().decode(errors='ignore').strip()
            except Exception:
                line = ""
            if line == "A":
                print(f"[Pico] ACK  (попытка {attempt})")
                return True
            if line == "N":
                print(f"[Pico] NAK  (попытка {attempt})")
                break

        print(f"[Pico] нет ответа (попытка {attempt})")

    return False


# ============================================================
#  КАМЕРА
# ============================================================
def open_webcam(index, width, height):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)   # Windows
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    for _ in range(5):
        cap.read()
    return cap


# ============================================================
#  ТОЧКИ
# ============================================================
points = []

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append((x, y))
        print(f"Точка {len(points)}: ({x}, {y})")


def draw_points(img):
    vis = img.copy()
    colors = [(0, 255, 255), (0, 255, 0), (255, 0, 0), (0, 0, 255)]
    names = ["LT", "RT", "RB", "LB"]
    for i, p in enumerate(points):
        cv2.circle(vis, p, 8, colors[i], -1)
        cv2.putText(vis, names[i], (p[0] + 10, p[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[i], 2)
    if len(points) >= 2:
        for i in range(len(points) - 1):
            cv2.line(vis, points[i], points[i + 1], (255, 255, 255), 1)
    if len(points) == 4:
        cv2.line(vis, points[3], points[0], (255, 255, 255), 1)
    hint = (f"Кликов: {len(points)}/4  LT→RT→RB→LB  "
            f"| s — старт, r — сброс, q — выход")
    cv2.putText(vis, hint, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return vis


# ============================================================
#  ЦВЕТА ЦЕНТРОВ
# ============================================================
def sample_cell_colors(rectified):
    h, w = rectified.shape[:2]
    cell_px = CELL_MM * PX_PER_MM
    colors = np.zeros((ROWS * COLS, 3), dtype=np.uint8)
    idx = 0
    for row in range(ROWS):
        for col in range(COLS):
            cx = int((col + 0.5) * cell_px)
            cy = int((row + 0.5) * cell_px)
            y0 = max(cy - SAMPLE_R, 0)
            y1 = min(cy + SAMPLE_R, h)
            x0 = max(cx - SAMPLE_R, 0)
            x1 = min(cx + SAMPLE_R, w)
            patch = rectified[y0:y1, x0:x1]
            if patch.size == 0:
                bgr = (0, 0, 0)
            else:
                bgr = patch.reshape(-1, 3).mean(axis=0).astype(np.uint8)
            colors[idx] = (int(bgr[2]), int(bgr[1]), int(bgr[0]))
            idx += 1
    return colors


def draw_centers(rectified, colors):
    out = rectified.copy()
    cell_px = CELL_MM * PX_PER_MM
    idx = 0
    for row in range(ROWS):
        for col in range(COLS):
            cx = int((col + 0.5) * cell_px)
            cy = int((row + 0.5) * cell_px)
            r, g, b = colors[idx]
            cv2.circle(out, (cx, cy), 4, (int(b), int(g), int(r)), -1)
            cv2.circle(out, (cx, cy), 5, (255, 255, 255), 1)
            idx += 1
    return out


# ============================================================
#  MAIN
# ============================================================
def main():
    cap = open_webcam(CAMERA_INDEX, CAM_WIDTH, CAM_HEIGHT)
    if cap is None:
        print(f"Камера #{CAMERA_INDEX} не открылась")
        return 1

    ser = open_uart(COM_PORT, BAUD)
    if ser is None:
        print("Работаю без UART")

    cv2.namedWindow("Camera")
    cv2.setMouseCallback("Camera", on_mouse)

    streaming = False
    print("Кликни 4 точки. Потом 's' — старт отправки с подтверждением.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord('q')):
                    break
                continue

            vis = draw_points(frame)

            if len(points) == 4:
                src = np.array(points, dtype=np.float32)
                if abs(cv2.contourArea(src.reshape(-1, 1, 2))) > 1.0:
                    H = cv2.getPerspectiveTransform(src, DST_QUAD)
                    rectified = cv2.warpPerspective(
                        frame, H, (FIELD_W_PX, FIELD_H_PX),
                        flags=cv2.INTER_LINEAR
                    )
                    colors = sample_cell_colors(rectified)
                    cv2.imshow("Rectified", draw_centers(rectified, colors))

                    if streaming and ser is not None:
                        ok = send_packet(ser, colors)
                        if not ok:
                            print("Пакет не подтверждён — пропуск")

            cv2.imshow("Camera", vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
            elif key == ord('r'):
                points.clear()
                streaming = False
                print("Точки сброшены")
            elif key == ord('s'):
                if len(points) == 4:
                    streaming = True
                    print("Старт отправки с ожиданием ACK")
                else:
                    print("Сначала 4 точки")

    except KeyboardInterrupt:
        print("\nПрервано")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
