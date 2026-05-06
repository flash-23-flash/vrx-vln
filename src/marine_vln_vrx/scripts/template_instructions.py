#!/usr/bin/env python3
"""Reference template instructions for Marine-VLN v1."""

TEMPLATE_INSTRUCTIONS = [
    "Go to waypoint A",
    "Pass between the red and green buoys",
    "Circle around the turtle clockwise",
    "Avoid the crocodile and stop near the dock",
    "First pass the gate, then go to the marker, finally stop",
]


if __name__ == "__main__":
    for i, text in enumerate(TEMPLATE_INSTRUCTIONS, start=1):
        print(f"{i}. {text}")
