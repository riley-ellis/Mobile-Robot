import csv
from rplidar import RPLidar
import numpy as np

def interpolate_scan(scan):
    """
    Interpolates a scan to have 360 distance measurements, one for each degree.
    """
    # Initialize an array to store distances for each degree
    full_scan = np.full(360, np.nan)

    for measurement in scan:
        _, angle, distance = measurement
        full_scan[int(angle)] = distance

    # Interpolate missing values
    angles = np.arange(360)
    distances = np.copy(full_scan)
    mask = np.isfinite(distances)
    distances[~mask] = np.interp(angles[~mask], angles[mask], distances[mask])

    return distances

def save_to_csv(filename, data):
    """
    Saves the scan data to a CSV file.
    """
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Scan', 'Angle', 'Distance'])
        for scan_number, scan in enumerate(data):
            for angle, distance in enumerate(scan):
                writer.writerow([scan_number, angle, distance])

def main():
    lidar = RPLidar('/dev/ttyUSB0')
    
    try:
        print("Collecting data. Press Ctrl+C to stop.")
        scans = []
        for i, scan in enumerate(lidar.iter_scans()):
            print("scan: ", scan)
            print("scan length: ", len(scan))
            interpolated_scan = interpolate_scan(scan)
            scans.append(interpolated_scan)
            if i >= 5:  # Collect 100 scans as an example
                break

        save_to_csv('lidar_data.csv', scans)
        print("Data collection complete. Saved to 'lidar_data.csv'.")

    except KeyboardInterrupt:
        print("Data collection stopped by user.")

    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == '__main__':
    main()
