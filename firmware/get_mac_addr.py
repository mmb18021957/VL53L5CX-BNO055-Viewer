import network
import ubinascii

def get_mac_address():
    # Create a WLAN object for the station interface (client mode)
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)  # Ensure the interface is active

    # Retrieve MAC address as bytes
    mac_bytes = wlan.config('mac')

    # Convert to uppercase hex string without separators
    mac_str = ubinascii.hexlify(mac_bytes).decode().upper()

    # Or with colon separators
    mac_colon = ':'.join(mac_str[i:i+2] for i in range(0, len(mac_str), 2))

    return mac_str, mac_colon

if __name__ == "__main__":
    mac_no_sep, mac_with_colon = get_mac_address()
    print("MAC (no separators):", mac_no_sep)
    print("MAC (with colons):", mac_with_colon)