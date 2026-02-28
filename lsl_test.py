# sender.py
# Run this on your main computer to send commands to the Raspberry Pi
# Install dependency: pip install pylsl

from pylsl import StreamInfo, StreamOutlet

info = StreamInfo(
    name='MyStream',
    type='Markers',
    channel_count=1,
    nominal_srate=0,
    channel_format='string',
    source_id='sender_001'
)

outlet = StreamOutlet(info)

print("LSL stream created. Waiting for receiver to connect...")
print()

COMMANDS = [
    "open", "close", "reverse open", "reverse close", "stop",
    "valve on", "valve off",
    "mastervalve on", "mastervalve off",
    "pumpin forward", "pumpin reverse", "pumpin stop",
    "pumpout forward", "pumpout reverse", "pumpout stop",
    "quit", "help"
]

def print_help():
    print("  SYSTEM COMMANDS:")
    print("    open          - Valves ON,  pumps forward")
    print("    close         - Valves OFF, pumps forward")
    print("    reverse open  - Valves ON,  pumps reversed")
    print("    reverse close - Valves OFF, pumps reversed")
    print("    stop          - Everything OFF")
    print()
    print("  VALVE:")
    print("    valve on          - Valve ON")
    print("    valve off         - Valve OFF")
    print()
    print("  MASTER VALVE:")
    print("    mastervalve on    - Master Valve ON")
    print("    mastervalve off   - Master Valve OFF")
    print()
    print("  PUMPIN:")
    print("    pumpin forward - Pumpin forward")
    print("    pumpin reverse - Pumpin reverse")
    print("    pumpin stop    - Pumpin OFF")
    print()
    print("  PUMPOUT:")
    print("    pumpout forward - Pumpout forward")
    print("    pumpout reverse - Pumpout reverse")
    print("    pumpout stop    - Pumpout OFF")
    print()
    print("  quit / help")
    print()

print_help()

while True:
    try:
        cmd = input("Enter command: ").strip().lower()

        if cmd == "":
            continue
        elif cmd == "help":
            print_help()
        elif cmd == "quit":
            outlet.push_sample(["quit"])
            print("Sent quit command. Exiting.")
            break
        elif cmd in COMMANDS:
            outlet.push_sample([cmd])
            print(f"Sent: {cmd}")
        else:
            print(f"Unknown command '{cmd}'. Type 'help' for options.")

    except KeyboardInterrupt:
        print("\nExiting sender.")
        break