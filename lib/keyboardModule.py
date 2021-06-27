from pynput import keyboard

_key = None

def on_press(key):
    global _key
    try:
        _key = key.char
    except AttributeError:
        _key = key

def on_release(key):
    global _key
    _key = None
    return False

def checkKey():
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release
    )
    listener.start()
    return _key