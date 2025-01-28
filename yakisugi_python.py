import cv2
import numpy as np
import time
from uvctypes import *
from queue import Queue
import csv
import requests
from datetime import datetime

# Buffer size for storing thermal frames
BUF_SIZE = 2
q = Queue(BUF_SIZE)

# Time initialization
start_time = time.time()
last_recorded_time = time.time()
temperature_data = []  # List to store time, temperature, and distance data

# Temperature threshold for alert
TEMPERATURE_THRESHOLD = 120.0

# Temperature range for filtering
TEMPERATURE_RANGE = (15.0, 130.0)  # Values in Celsius

# Interval for saving data (in seconds)
SAVE_INTERVAL = 5

# Path to the university logo
LOGO_PATH = "/home/charlie/Desktop/iaac/term02/thermal-camera/files/IAACLOGO.png"
logo_img = cv2.imread(LOGO_PATH, cv2.IMREAD_UNCHANGED)

if logo_img is not None and logo_img.shape[2] == 4:
    # Convert RGBA to RGB if necessary
    logo_img = cv2.cvtColor(logo_img, cv2.COLOR_BGRA2BGR)
elif logo_img is None:
    print(f"Warning: Unable to load the logo from {LOGO_PATH}. Check the file path.")
else:
    print("Logo successfully loaded.")

# ESP32 configuration
ESP32_IP = "http://192.168.4.1"  # Replace with your ESP32 IP

def py_frame_callback(frame, userptr):
    """Callback to capture thermal frames."""
    try:
        array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
        data = np.frombuffer(array_pointer.contents, dtype=np.uint16).reshape(frame.contents.height, frame.contents.width)

        if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
            print("Corrupted or incomplete data received.")
            return

        if not q.full():
            q.put(data)
    except Exception as e:
        print(f"Error in callback: {e}")

PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(py_frame_callback)

def ktoc(val):
    """Converts Kelvin to Celsius."""
    return (val - 27315) / 100.0

def format_time(seconds):
    """Converts time in seconds to HH:MM:SS format."""
    hrs = int(seconds // 3600)
    mins = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    return f"{hrs:02}:{mins:02}:{secs:02}"

def get_distance_from_esp32():
    """Fetch distance data from the ESP32."""
    try:
        response = requests.get(ESP32_IP, timeout=1)
        if response.status_code == 200:
            data = response.json()
            return data.get("distance", None)
        else:
            print(f"Error fetching data from ESP32: {response.status_code}")
            return None
    except requests.RequestException as e:
        print(f"Error connecting to ESP32: {e}")
        return None

def filter_and_display_temperature(thermal_data):
    """Applies a filter for the temperature range and generates a thermal image."""
    thermal_data_celsius = ktoc(thermal_data)

    # Create a mask for temperatures within the range
    min_temp, max_temp = TEMPERATURE_RANGE
    mask = (thermal_data_celsius >= min_temp) & (thermal_data_celsius <= max_temp)

    # Normalize the thermal data
    normalized_data = np.clip((thermal_data_celsius - min_temp) / (max_temp - min_temp), 0, 1)
    normalized_data = (normalized_data * 255).astype(np.uint8)

    # Apply a classic color map (red to blue via green and yellow)
    img_color = cv2.applyColorMap(normalized_data, cv2.COLORMAP_JET)

    # Convert out-of-range data to grayscale
    gray_data = cv2.normalize(thermal_data_celsius, None, 0, 255, cv2.NORM_MINMAX)
    gray_data = np.uint8(gray_data)
    img_gray = cv2.cvtColor(gray_data, cv2.COLOR_GRAY2BGR)
    img_color[~mask] = img_gray[~mask]

    return img_color, mask, thermal_data_celsius

def draw_interface(img, min_temp, max_temp, distance):
    """Draws the user interface with temperature and distance data."""
    overlay = img.copy()
    cv2.rectangle(overlay, (10, 10), (250, 120), (0, 0, 0), -1)
    alpha = 0.6
    img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    # Display temperature values
    cv2.putText(img, f"Max Temp: {max_temp:.1f}C", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)
    cv2.putText(img, f"Min Temp: {min_temp:.1f}C", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)

    # Display distance
    cv2.putText(img, f"Distance: {distance} cm", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Draw gradient color bar (smaller size)
    gradient = np.linspace(TEMPERATURE_RANGE[1], TEMPERATURE_RANGE[0], 100).astype(np.float32)  # Smaller bar
    gradient_normalized = np.clip((gradient - TEMPERATURE_RANGE[0]) / (TEMPERATURE_RANGE[1] - TEMPERATURE_RANGE[0]), 0, 1)
    gradient_normalized = (gradient_normalized * 255).astype(np.uint8)
    gradient_color = cv2.applyColorMap(gradient_normalized[:, None], cv2.COLORMAP_JET)

    # Create gradient bar with white border
    x_start = img.shape[1] - 100  # Reduce size of the bar
    gradient_with_border = np.zeros((120, 50, 3), dtype=np.uint8)
    gradient_with_border[10:-10, 10:40] = gradient_color
    cv2.rectangle(gradient_with_border, (10, 10), (40, 100), (255, 255, 255), 1)

    # Insert the gradient bar
    img[10:130, x_start:x_start + 50] = gradient_with_border

    # Display temperature scale values
    cv2.putText(img, f"{TEMPERATURE_RANGE[1]:.1f}C", (x_start + 5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, f"{TEMPERATURE_RANGE[0]:.1f}C", (x_start + 5, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Insert the logo
    if logo_img is not None:
        try:
            logo_resized = cv2.resize(logo_img, (80, 40))
            h, w = logo_resized.shape[:2]
            y_offset = img.shape[0] - h - 10
            x_offset = 10
            img[y_offset:y_offset + h, x_offset:x_offset + w] = logo_resized
        except Exception as e:
            print(f"Error inserting the logo: {e}")

    return img

def draw_alert(img, message):
    """Draws an alert message in the center of the screen."""
    height, width = img.shape[:2]
    text_size = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)[0]
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2
    cv2.putText(img, message, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

def select_roi_live():
    """Allows ROI selection in live view."""
    while True:
        data = q.get(True, 500)
        resized_data = cv2.resize(data, (640, 480))
        img_color, _, _ = filter_and_display_temperature(resized_data)

        # Display live feed
        cv2.putText(img_color, "Press 's' to select ROI or 'q' to quit", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("Thermal Camera - Live View", img_color)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            # Allow ROI selection
            roi = cv2.selectROI("Thermal Camera - Live View", img_color, fromCenter=False, showCrosshair=True)
            if roi != (0, 0, 0, 0):
                print(f"ROI selected: {roi}")
                cv2.destroyWindow("Thermal Camera - Live View")
                return roi
            else:
                print("Invalid ROI. Please select again.")
        elif key == ord('q'):
            print("Exiting...")
            cv2.destroyAllWindows()
            exit(0)

def main():
    global start_time, last_recorded_time, temperature_data

    ctx = POINTER(uvc_context)()
    dev = POINTER(uvc_device)()
    devh = POINTER(uvc_device_handle)()
    ctrl = uvc_stream_ctrl()

    res = libuvc.uvc_init(byref(ctx), 0)
    if res < 0:
        print("Error initializing UVC.")
        exit(1)

    try:
        res = libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
        if res < 0:
            print("Device not found. Make sure it is connected.")
            exit(1)

        try:
            res = libuvc.uvc_open(dev, byref(devh))
            if res < 0:
                print("Error opening the device.")
                exit(1)

            print("Device connected and opened.")

            frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
            if not frame_formats:
                print("The device does not support the Y16 format.")
                exit(1)

            libuvc.uvc_get_stream_ctrl_format_size(
                devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,
                frame_formats[0].wWidth, frame_formats[0].wHeight,
                int(1e7 / frame_formats[0].dwDefaultFrameInterval)
            )

            res = libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0)
            if res < 0:
                print("Error starting streaming.")
                exit(1)

            try:
                roi = select_roi_live()
                cv2.namedWindow("Thermal Camera", cv2.WINDOW_NORMAL)

                while True:
                    data = q.get(True, 500)
                    resized_data = cv2.resize(data, (640, 480))

                    img_color, mask, thermal_data_celsius = filter_and_display_temperature(resized_data)

                    # Crop to ROI first
                    x, y, w, h = map(int, roi)
                    cropped_data = resized_data[y:y + h, x:x + w]
                    img_color, mask, thermal_data_celsius = filter_and_display_temperature(cropped_data)

                    # Draw interface within ROI
                    min_temp, max_temp = thermal_data_celsius.min(), thermal_data_celsius.max()
                    distance = get_distance_from_esp32()
                    img_color = draw_interface(img_color, min_temp, max_temp, distance)

                    # Check for temperature threshold
                    if max_temp > TEMPERATURE_THRESHOLD:
                        draw_alert(img_color, "MUEVETEEE!")

                    # Show the result
                    cv2.imshow("Thermal Camera", img_color)

                    # Save data at intervals
                    if time.time() - last_recorded_time >= SAVE_INTERVAL:
                        elapsed_time = time.time() - start_time
                        temperature_data.append({
                            "time": format_time(elapsed_time),
                            "min_temp": min_temp,
                            "max_temp": max_temp,
                            "distance": distance
                        })
                        print(f"Data recorded: {temperature_data[-1]}")
                        last_recorded_time = time.time()

                    # Quit loop if 'q' is pressed
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("Exiting...")
                        break

                cv2.destroyAllWindows()

                # Ask to save data
                save_data = input("Do you want to save the recorded data? (y/n): ").strip().lower()
                if save_data == 'y':
                    with open("temperature_distance_data.csv", "w", newline="") as csvfile:
                        fieldnames = ["time", "min_temp", "max_temp", "distance"]
                        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                        writer.writeheader()
                        writer.writerows(temperature_data)
                    print("Data saved to 'temperature_distance_data.csv'.")
                else:
                    print("Data was not saved.")

            finally:
                libuvc.uvc_stop_streaming(devh)

        finally:
            libuvc.uvc_unref_device(dev)
    finally:
        libuvc.uvc_exit(ctx)

if __name__ == '__main__':
    main()
